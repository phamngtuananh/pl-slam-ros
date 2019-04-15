echo "Building 3rdparty/line_descriptor ... "
cd dependencies/stvo-pl/3rdparty/line_descriptor
mkdir build
cd build
cmake ..
make -j4
cd ../../../

echo "Building StVO-PL ... "
mkdir build
cd build
cmake ..
make -j4
cd ../../pl-slam/

echo "Building 3rdparty/DBoW2 ... "
cd 3rdparty/DBoW2
mkdir build
cd build
cmake ..
make -j4
cd ../../../

echo "Uncompressing vocabulary ..."
cd vocabulary
tar -xf voc.tar.gz
cd ..

echo "Building PL-SLAM ... "
mkdir build
cd build
cmake ..
make -j4

echo "Finished building dependencies."