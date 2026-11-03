"""
ds_prepare.py -- 把采集的数据集转成 YOLOv4-tiny 训练工程要的 VOC 结构

输入:  dataset/shot_XXXX.bmp + shot_XXXX.xml  (labelImg 直接产出的扁平目录)
输出:  VOCdevkit/VOC2007/{JPEGImages, Annotations, ImageSets/Main}

做三件事:
  1. BMP -> JPG。训练工程的 dataloader 用 PIL 按扩展名找图, 而且
     BMP 一张 1.1 MB, 400 张就 440 MB, 每个 epoch 都要全部读一遍,
     转成 JPG 后约 1/10 大小, 训练明显更快。
     用 quality=95, 视觉无损, 对检测精度没有可测影响。
  2. 过滤。没有 xml 的(废图)、xml 里零目标的, 都跳过。
  3. 划分 train/val。按文件名哈希划分而不是随机 --
     重复运行结果一致, 便于复现和对比实验。

注意: 不改动原始 dataset 目录, 只读。
"""
import os
import sys
import glob
import struct
import shutil
import hashlib
import xml.etree.ElementTree as ET


def read_bmp(path):
    """读 24 位 BMP -> (w, h, rgb bytes)。只处理 capture.cpp 产出的格式。"""
    with open(path, 'rb') as f:
        raw = f.read()
    if raw[:2] != b'BM':
        raise ValueError('not BMP: %s' % path)
    off = struct.unpack_from('<I', raw, 10)[0]
    w = struct.unpack_from('<i', raw, 18)[0]
    h = struct.unpack_from('<i', raw, 22)[0]
    bpp = struct.unpack_from('<H', raw, 28)[0]
    if bpp != 24:
        raise ValueError('expect 24bpp, got %d' % bpp)
    row_raw = w * 3
    pad = (4 - (row_raw & 3)) & 3
    row = row_raw + pad
    # BMP: 自下而上 + BGR
    out = bytearray(w * h * 3)
    for y in range(h):
        s = off + y * row
        d = (h - 1 - y) * row_raw
        line = raw[s:s + row_raw]
        out[d:d + row_raw] = line
    return w, h, bytes(out), 'BGR'


def count_objects(xml_path):
    try:
        root = ET.parse(xml_path).getroot()
    except Exception:
        return -1
    return len(root.findall('object'))


def fix_xml(xml_path, out_path, new_filename):
    """把 xml 里的 filename/path 改成转换后的 jpg 名。
    训练工程只按 ImageSets 里的 stem 找图, filename 字段其实不用,
    但改掉可以避免以后用别的工具时对不上。"""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    fn = root.find('filename')
    if fn is not None:
        fn.text = new_filename
    p = root.find('path')
    if p is not None:
        p.text = new_filename
    tree.write(out_path, encoding='utf-8')


def split_key(stem):
    """按名字哈希决定 train/val, 保证可复现"""
    h = hashlib.md5(stem.encode('utf-8')).hexdigest()
    return int(h[:8], 16) / float(0xFFFFFFFF)


def main(src, dst, val_ratio=0.1):
    try:
        from PIL import Image
    except ImportError:
        print('need Pillow:  pip install pillow')
        return 1

    jpeg_dir = os.path.join(dst, 'VOC2007', 'JPEGImages')
    ann_dir = os.path.join(dst, 'VOC2007', 'Annotations')
    set_dir = os.path.join(dst, 'VOC2007', 'ImageSets', 'Main')
    for p in (jpeg_dir, ann_dir, set_dir):
        os.makedirs(p, exist_ok=True)

    bmps = sorted(glob.glob(os.path.join(src, '*.bmp')))
    print('found %d bmp in %s' % (len(bmps), src))

    kept, skip_noann, skip_empty, failed = [], [], [], []

    for i, bp in enumerate(bmps):
        stem = os.path.splitext(os.path.basename(bp))[0]
        xp = os.path.join(src, stem + '.xml')

        if not os.path.exists(xp):
            skip_noann.append(stem)
            continue
        n = count_objects(xp)
        if n <= 0:
            skip_empty.append(stem)
            continue

        try:
            w, h, buf, order = read_bmp(bp)
            if order == 'BGR':
                img = Image.frombytes('RGB', (w, h), buf)
                b, g, r = img.split()
                img = Image.merge('RGB', (r, g, b))
            else:
                img = Image.frombytes('RGB', (w, h), buf)
            img.save(os.path.join(jpeg_dir, stem + '.jpg'),
                     quality=95, subsampling=0)
            fix_xml(xp, os.path.join(ann_dir, stem + '.xml'), stem + '.jpg')
            kept.append(stem)
        except Exception as e:
            failed.append((stem, str(e)))

        if (i + 1) % 50 == 0:
            print('  %d/%d ...' % (i + 1, len(bmps)))

    # 划分
    train, val = [], []
    for s in kept:
        (val if split_key(s) < val_ratio else train).append(s)

    def dump(name, lst):
        with open(os.path.join(set_dir, name), 'w') as f:
            f.write('\n'.join(sorted(lst)) + '\n')

    dump('train.txt', train)
    dump('val.txt', val)
    dump('trainval.txt', kept)
    dump('test.txt', val)       # 本项目不单独留 test, 用 val 评估

    print()
    print('=' * 56)
    print('converted  %d' % len(kept))
    print('  train    %d' % len(train))
    print('  val      %d' % len(val))
    print('skipped (no annotation, i.e. rejected shots)  %d' % len(skip_noann))
    print('skipped (annotation with zero objects)        %d' % len(skip_empty))
    if failed:
        print('FAILED %d:' % len(failed))
        for s, e in failed[:10]:
            print('   %s: %s' % (s, e))
    print()
    print('output: %s' % dst)
    print('  VOC2007/JPEGImages    %d jpg' % len(os.listdir(jpeg_dir)))
    print('  VOC2007/Annotations   %d xml' % len(os.listdir(ann_dir)))
    print('  VOC2007/ImageSets/Main/{train,val,trainval,test}.txt')
    return 0


if __name__ == '__main__':
    src = sys.argv[1] if len(sys.argv) > 1 else 'E:/dataset'
    dst = sys.argv[2] if len(sys.argv) > 2 else 'VOCdevkit'
    sys.exit(main(src, dst))
