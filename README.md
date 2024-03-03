- [x] 适配 open_vio 输出的关键帧信息
- [x] 带入尺度的三维重建

## 1. 编译
```shell
cmake --build build   \
    -DCUDA_ENABLED=ON \
    -DCUDA_NVCC_FLAGS="--std c++14" && \
    make -j$(( $(nproc) + 1 ))
```

## 2. 图像间的特征匹配
输入: 图像文件夹，检索文件，匹配策略类型
输出: 特征提取结果，特征匹配结果
```shell
# ./bin/run_matching images_path retrieval_path matching_type output_path
./bin/run_matching ~/output/images/ ~/output/pair.txt retrieval ~/output/
```

## 3. 稀疏重建
输入: 特征提取结果，特征匹配结果，相机内参文件
输出: 重建结果
```shell
./bin/rec_euroc ~/output/ ~/output/map/
```

## Reference
1. https://github.com/openxrlab/xrsfm
2. https://github.com/colmap/colmap