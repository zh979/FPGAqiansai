#pragma GCC optimize(3, "inline")

#include "laser.h"
#include "yolo_post.h"

void ls_init(Laser* l, const LaserZone* zone)
{
    l->state = LS_IDLE;
    l->on = 0;

    if (zone) {
        l->zone = *zone;
    } else {
        l->zone.x0 = l->zone.y0 = 0.0f;
        l->zone.x1 = l->zone.y1 = 0.0f;
    }

    l->aim_tol_px     = 30.0f;
    l->aim_hold_ticks = 1;

    l->fire_ticks     = 1;
    l->cool_ticks     = 2;
    l->min_score      = 0.6f;
    l->done_radius_px = 40.0f;

    l->fire_coast_max = 2;

    l->jump_tol_px    = 120.0f;

    l->done_max_age_ms = 4000;

    l->vel_deadband = 30.0f;

    l->aim_cnt = l->fire_cnt = l->cool_cnt = 0;
    l->fire_coast = 0;
    l->tgt_x = l->tgt_y = 0.0f;
    l->done_n = 0;
    l->done_head = 0;
    for (int i = 0; i < LZ_MAX_DONE; i++) {
        l->done_uid[i] = 0;
        l->done_vx[i] = l->done_vy[i] = 0.0f;
        l->done_t[i] = 0;
    }
    l->fire_done_idx = -1;
    l->now_ms = 0;
    l->just_done = 0;
    l->blk_why = LB_NONE;
    l->n_pulse = 0;
    l->n_fired = 0;
    l->n_blocked = 0;
}

void ls_clear_done(Laser* l)
{
    l->done_n = 0;
    l->done_head = 0;
    for (int i = 0; i < LZ_MAX_DONE; i++)
        l->done_uid[i] = 0;
    l->fire_done_idx = -1;
}

const char* ls_block_name(int why)
{
    switch (why) {
    case LB_NONE:  return "--  ";
    case LB_NOTGT: return "NOTG";
    case LB_PRED:  return "PRED";
    case LB_CLASS: return "CLAS";
    case LB_SCORE: return "SCOR";
    case LB_ZONE:  return "ZONE";
    case LB_DONE:  return "DONE";
    case LB_JUMP:  return "JUMP";
    case LB_AIM:   return "AIM!";
    }
    return "????";
}

static int in_zone(const Laser* l, float x, float y)
{
    return x >= l->zone.x0 && x <= l->zone.x1 &&
           y >= l->zone.y0 && y <= l->zone.y1;
}

static int already_done(const Laser* l, float x, float y, int uid,
                        unsigned long now_ms, int skip_idx)
{
    const float r2 = l->done_radius_px * l->done_radius_px;
    for (int i = 0; i < l->done_n; i++) {
        if (i == skip_idx)
            continue;
        if (uid > 0 && l->done_uid[i] == uid)
            return 1;

        unsigned long age = (now_ms > l->done_t[i]) ? (now_ms - l->done_t[i]) : 0;
        if (age > l->done_max_age_ms)
            age = l->done_max_age_ms;
        float dt = (float)age * 0.001f;

        float px = l->done_x[i] + l->done_vx[i] * dt;
        float py = l->done_y[i] + l->done_vy[i] * dt;
        float dx = x - px;
        float dy = y - py;
        if (dx * dx + dy * dy < r2)
            return 1;
    }
    return 0;
}

int ls_is_done(const Laser* l, float x, float y)
{
    return already_done(l, x, y, 0, l->now_ms, -1);
}

static int mark_done(Laser* l, float x, float y, float vx, float vy,
                     int uid, unsigned long now_ms)
{
    if (vx * vx + vy * vy < l->vel_deadband * l->vel_deadband)
        vx = vy = 0.0f;

    int idx = l->done_head;
    l->done_x[idx] = x;
    l->done_y[idx] = y;
    l->done_vx[idx] = vx;
    l->done_vy[idx] = vy;
    l->done_t[idx] = now_ms;
    l->done_uid[idx] = uid;
    l->done_head = (idx + 1) % LZ_MAX_DONE;
    if (l->done_n < LZ_MAX_DONE)
        l->done_n++;
    return idx;
}

static void undo_done(Laser* l)
{
    int idx = l->fire_done_idx;
    if (idx < 0 || l->done_n <= 0)
        return;

    int last = (l->done_head - 1 + LZ_MAX_DONE) % LZ_MAX_DONE;
    if (idx != last)
        return;

    l->done_head = last;
    l->done_n--;
    l->done_uid[last] = 0;
    l->fire_done_idx = -1;
}

void ls_abort(Laser* l)
{
    l->state = LS_IDLE;
    l->on = 0;
    l->aim_cnt = l->fire_cnt = 0;
    l->fire_coast = 0;
    l->fire_done_idx = -1;
}

int ls_tick(Laser* l, int has_target, int measured, int class_id,
            float score, float tx, float ty, float aim_err_px, int uid,
            float vx, float vy, unsigned long now_ms)
{
    l->just_done = 0;
    l->now_ms = now_ms;

    if (l->state == LS_COOLDOWN) {
        l->on = 0;
        if (++l->cool_cnt >= l->cool_ticks) {
            l->cool_cnt = 0;
            l->state = LS_IDLE;
        }
        return 0;
    }

    int ok = has_target;
    int why = ok ? LB_NONE : LB_NOTGT;

    if (ok && !measured) {
        if (l->state == LS_FIRING && l->fire_coast < l->fire_coast_max)
            l->fire_coast++;
        else
            { ok = 0; why = LB_PRED; }
    } else if (ok) {
        l->fire_coast = 0;
    }

    if (ok && class_id != YP_CLS_WEED)
        { ok = 0; why = LB_CLASS; }

    if (ok && score < l->min_score)
        { ok = 0; why = LB_SCORE; }

    if (ok && !in_zone(l, tx, ty))
        { ok = 0; why = LB_ZONE; }

    if (ok && already_done(l, tx, ty, uid, now_ms,
                           (l->state == LS_FIRING) ? l->fire_done_idx : -1))
        { ok = 0; why = LB_DONE; }

    if (!ok) {
        if (l->state != LS_IDLE)
            l->n_blocked++;
        l->blk_why = why;

        if (l->state == LS_FIRING && why != LB_DONE)
            undo_done(l);
        ls_abort(l);
        return 0;
    }

    l->blk_why = LB_NONE;

    switch (l->state) {
    case LS_IDLE:
        l->tgt_x = tx;
        l->tgt_y = ty;
        l->aim_cnt = 0;
        l->state = LS_AIMING;
        l->on = 0;
        break;

    case LS_AIMING:
        l->on = 0;
        l->tgt_x = tx;
        l->tgt_y = ty;

        if (aim_err_px <= l->aim_tol_px) {
            if (++l->aim_cnt >= l->aim_hold_ticks) {
                l->state = LS_FIRING;
                l->on = 1;

                l->n_pulse++;
                l->fire_done_idx = mark_done(l, tx, ty, vx, vy, uid, now_ms);
                l->fire_cnt = 1;

                if (l->fire_cnt >= l->fire_ticks) {
                    l->n_fired++;
                    l->just_done = 1;
                    l->fire_cnt = 0;
                    l->cool_cnt = 0;
                    l->fire_done_idx = -1;
                    l->state = LS_COOLDOWN;
                }
            }
        } else {
            l->aim_cnt = 0;
        }
        break;

    case LS_FIRING:
        l->on = 1;

        {
            float jx = tx - l->tgt_x;
            float jy = ty - l->tgt_y;
            if (jx * jx + jy * jy > l->jump_tol_px * l->jump_tol_px) {
                l->n_blocked++;
                l->blk_why = LB_JUMP;

                undo_done(l);
                ls_abort(l);
                break;
            }

            l->tgt_x = tx;
            l->tgt_y = ty;
        }

        if (aim_err_px > l->aim_tol_px * 2.0f) {
            l->n_blocked++;
            l->blk_why = LB_AIM;
            undo_done(l);
            ls_abort(l);
            break;
        }
        if (++l->fire_cnt >= l->fire_ticks) {
            l->fire_done_idx = -1;
            l->n_fired++;

            l->just_done = 1;
            l->on = 0;
            l->fire_cnt = 0;
            l->cool_cnt = 0;
            l->state = LS_COOLDOWN;
        }
        break;

    default:
        ls_abort(l);
        break;
    }

    return l->on;
}

const char* ls_state_name(LaserState s)
{
    switch (s) {
    case LS_IDLE:     return "IDLE";
    case LS_AIMING:   return "AIM ";
    case LS_FIRING:   return "FIRE";
    case LS_COOLDOWN: return "COOL";
    }
    return "????";
}
