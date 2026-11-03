"""
ds_config.py -- 把训练工程配置成 2 类小数据集的参数

在 train 目录下运行。只改必要的几处, 其余保持原作者的默认值。

为什么要改这些 (数据集只有 380 张, 默认参数是按 VOC 的 1.7 万张设的):

  model_path        必须加载 COCO 预训练权重。从零训 380 张学不出
                    任何有用的特征 -- backbone 的低层边缘/纹理检测器
                    需要几万张才能收敛。
  Freeze_Train      开。先冻结 backbone 只训检测头, 让随机初始化的
                    头部先稳定下来, 再解冻微调。不冻结的话, 头部的
                    大梯度会在前几个 batch 就破坏预训练的 backbone。
  UnFreeze_Epoch    300 -> 100。380 张图 / batch 8 = 42 iter/epoch,
                    100 epoch 已经 4200 次迭代, 对 2 类任务足够。
                    继续训只会过拟合。
  batch_size        32 -> 8/4。380 张用 batch 32 时一个 epoch 只有
                    12 次梯度更新, BN 统计也不稳。小 batch 更新次数多。
  Init_lr           1e-2 -> 1e-3。小 batch 要配小学习率, 否则梯度噪声
                    会让损失震荡。也因为是微调而非从零训。
  mosaic            开。380 张太少, mosaic 把 4 张拼成 1 张, 等效扩充
                    数据量并制造大量"目标在不同位置/尺度"的样本,
                    正好补上我们单株定位组覆盖不到的组合。
  optimizer         sgd -> adam。小数据集上 adam 收敛更快且对 lr
                    不那么敏感, 省去反复调参。
"""
import os
import re
import sys
import io


def patch(path, rules, label):
    if not os.path.exists(path):
        print('  !! %s not found' % path)
        return False
    t = io.open(path, encoding='utf-8', errors='replace').read()
    orig = t
    done = []
    for pat, rep, note in rules:
        new, n = re.subn(pat, rep, t, count=1, flags=re.M)
        if n:
            t = new
            done.append(note)
        else:
            print('  ?  no match: %s' % note)
    if t != orig:
        io.open(path, 'w', encoding='utf-8').write(t)
    print('  %s: %d changes' % (label, len(done)))
    for d in done:
        print('      %s' % d)
    return True


def main():
    # ---- classes.txt ----
    os.makedirs('logs', exist_ok=True)
    with open('logs/classes.txt', 'w') as f:
        f.write('weed\ncrop\n')
    print('wrote logs/classes.txt  (weed=0, crop=1)')
    print('  !! order matters: code hardcodes YP_CLS_WEED=0')
    print()

    # ---- voc_annotation.py ----
    # annotation_mode=2: 只根据已有的 ImageSets 生成 2007_train/val.txt,
    # 不重新划分 -- 划分已经由 ds_prepare.py 按哈希做好了, 保证可复现。
    patch('voc_annotation.py', [
        (r'^annotation_mode\s*=.*$', 'annotation_mode     = 2',
         'annotation_mode = 2 (use existing ImageSets, do not re-split)'),
    ], 'voc_annotation.py')
    print()

    # ---- train.py ----
    rules = [
        (r'^(\s*)model_path\s*=\s*[\'"][\'"].*$',
         r"\1model_path      = 'model_data/yolov4_tiny_weights_coco.pth'",
         'model_path -> COCO pretrained (essential for 380 images)'),
        (r'^(\s*)Freeze_Train\s*=\s*False.*$',
         r'\1Freeze_Train        = True',
         'Freeze_Train = True (protect pretrained backbone early on)'),
        (r'^(\s*)Freeze_Epoch\s*=\s*50.*$',
         r'\1Freeze_Epoch        = 30',
         'Freeze_Epoch = 30'),
        (r'^(\s*)UnFreeze_Epoch\s*=\s*300.*$',
         r'\1UnFreeze_Epoch      = 100',
         'UnFreeze_Epoch 300 -> 100 (380 imgs would overfit)'),
        (r'^(\s*)Freeze_batch_size\s*=\s*32.*$',
         r'\1Freeze_batch_size   = 8',
         'Freeze_batch_size 32 -> 8 (more updates per epoch)'),
        (r'^(\s*)Unfreeze_batch_size\s*=\s*32.*$',
         r'\1Unfreeze_batch_size = 4',
         'Unfreeze_batch_size 32 -> 4'),
        (r'^(\s*)Init_lr\s*=\s*1e-2.*$',
         r'\1Init_lr             = 1e-3',
         'Init_lr 1e-2 -> 1e-3 (small batch + finetune)'),
        (r'^(\s*)optimizer_type\s*=\s*"sgd".*$',
         r'\1optimizer_type      = "adam"',
         'optimizer sgd -> adam (faster on small data)'),
        (r'^(\s*)mosaic\s*=\s*False.*$',
         r'\1mosaic              = True',
         'mosaic = True (data augmentation, critical at 380 imgs)'),
        (r'^(\s*)eval_period\s*=\s*10.*$',
         r'\1eval_period         = 5',
         'eval_period 10 -> 5 (watch mAP more closely)'),
        (r'^(\s*)save_period\s*=\s*10.*$',
         r'\1save_period         = 25',
         'save_period 10 -> 25 (fewer checkpoints)'),
        (r'^(\s*)num_workers\s*=\s*4.*$',
         r'\1num_workers         = 2',
         'num_workers 4 -> 2 (safer on Windows)'),
    ]
    patch('train.py', rules, 'train.py')

    print()
    print('=' * 58)
    print('remaining manual step:')
    print('  download COCO pretrained weights to')
    print('      model_data/yolov4_tiny_weights_coco.pth')
    print('  it is in tools/predictor/ of the accelerator archive')
    print()
    print('then:')
    print('  python voc_annotation.py     # -> 2007_train.txt / 2007_val.txt')
    print('  python train.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
