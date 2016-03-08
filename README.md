SICP-test
=========

This is simple ICP registration codes by using sparse ICP (SICP) and PCL 1.7 APIs.

Requirements
------------

### sparseicp
- http://lgg.epfl.ch/sparseicp
- https://code.google.com/p/sparseicp/
- https://github.com/OpenGP/sparseicp 

(now nanoflann.h is included in sparseicp)

Then apply the patch. (see the Build section)


### misc
- pcl > 1.7
- cmake > 2.8
- eigen > 3


Build
-----
```
git clone https://github.com/tttamaki/SICP-test.git
git clone https://github.com/OpenGP/sparseicp.git
echo "apply the patch, if needed"
patch -p0 <<EOF
diff --git sparseicp.org/ICP.h sparseicp/ICP.h
index b9583f8..8013642 100644
--- sparseicp.org/ICP.h
+++ sparseicp/ICP.h
@@ -345,7 +345,7 @@ namespace SICP {
                     if(dual < par.stop) break;
                 }
                 /// C update (lagrange multipliers)
-                Eigen::VectorXf P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
+                Eigen::VectorXd P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
                 if(!par.use_penalty) C.noalias() += mu*P;
                 /// mu update (penalty)
                 if(mu < par.max_mu) mu *= par.alpha;
EOF
echo "patch end."
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

Run
---

sample 1
```
cd build
./sicp1_simple_icp ../data/bunny/bun{000,045}mesh.ply
```

sample 2
```
cd build
./sicp2_with_normal_iterative_view ../data/bunny/bun{000,045}mesh.ply
```



