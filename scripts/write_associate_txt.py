import os

asso_file = '/home/cxt/Documents/ORB_SLAM3/Examples/RGB-D/associations/trans_set2_scene2.txt'

rgb_d_path = '/media/cxt/HUIJIE/dataset/Transparentdataset/trans_data/set2/scene2/rgb'

rgb_files = sorted(os.listdir(rgb_d_path))
d_files = set(os.listdir(rgb_d_path.replace('rgb', 'depth')))

p_name = -4
p_s = -(4+9)
with open(asso_file, 'w') as af:
    for f in rgb_files:
        if f not in d_files:
            print(f)
            continue
        af.write('{}.{} rgb/{} {}.{} depth/{}\n'.format(f[:p_s], f[p_s:p_name], f, f[:p_s], f[p_s:p_name], f))