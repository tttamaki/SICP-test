SICP-test
=========

This is simple ICP registration codes by using sparse ICP (SICP) and PCL 1.7 APIs.

Requirements
------------

### sparseicp
- http://lgg.epfl.ch/sparseicp
- https://code.google.com/p/sparseicp/
- https://code.google.com/p/sparseicp/ 

### nanoflann
- https://github.com/jlblancoc/nanoflann

### misc
- pcl > 1.7
- cmake > 2.8
- eigen > 3


Build
-----
```
git clone https://github.com/tttamaki/SICP-test.git
git clone https://github.com/jlblancoc/nanoflann.git
git clone https://github.com/OpenGP/sparseicp.git
cd SICP-test/
mkdir build
cd build/
cmake ..
make
```




Codes
-----
- `sicp1_simple_icp.cpp`: simple ICP/SICP registration with sparseicp library, along with pcl for reading and visualizaton.
- `sicp2_with_normal_iterative_view.cpp`: ICP/SICP with point-to-point/point-to-plane. Registration process is animated.

