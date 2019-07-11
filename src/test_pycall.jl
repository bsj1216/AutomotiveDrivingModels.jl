# ============================
# TESTBED
# ============================

using PyCall
pushfirst!(PyVector(pyimport("sys")."path"),"/home/sbae/automotive-control-temporary/python/nnmpc/sgan")
sganPredictor = pyimport("sgan.predictor")
model_path = "/home/sbae/automotive-control-temporary/python/nnmpc/sgan/models/sgan-models/eth_8_model.pt"
predictor = sganPredictor.Predictor(model_path)

data = [
          [ 1,  1.000e+00,  8.460e+00,  3.590e+00],
          [ 1,  2.000e+00,  1.364e+01,  5.800e+00],
          [ 2,  1.000e+00,  9.570e+00,  3.790e+00],
          [ 2,  2.000e+00,  1.364e+01,  5.800e+00],
          [ 3,  1.000e+00,  1.067e+01,  3.990e+00],
          [ 3,  2.000e+00,  1.364e+01,  5.800e+00],
          [ 4,  1.000e+00,  1.173e+01,  4.320e+00],
          [ 4,  2.000e+00,  1.209e+01,  5.750e+00],
          [ 5,  1.000e+00,  1.281e+01,  4.610e+00],
          [ 5,  2.000e+00,  1.137e+01,  5.800e+00],
          [ 6,  1.000e+00,  1.281e+01,  4.610e+00],
          [ 6,  2.000e+00,  1.031e+01,  5.970e+00],
          [ 7,  1.000e+00,  1.194e+01,  6.770e+00],
          [ 7,  2.000e+00,  9.570e+00,  6.240e+00],
          [ 8,  1.000e+00,  1.103e+01,  6.840e+00],
          [ 8,  2.000e+00,  8.730e+00,  6.340e+00]
          ]

# for i in 1:length(data)
#       println(data[i])
# end
# println(data)

# data = [[1.0, 1.0, 9.0, 0.0],
#        [1.0, 2.0, 3.0, 0.0],
#        [1.0, 3.0, 3.0, -3.0],
#        [1.0, 4.0, 9.0, -3.0],
#        [1.0, 5.0, 15.0, -3.0],
#        [1.0, 6.0, 3.0, -6.0],
#        [1.0, 7.0, 9.0, -6.0],
#        [2.0, 1.0, 10.9476, -0.161575],
#        [2.0, 2.0, 3.82293, 0.0],
#        [2.0, 3.0, 3.82293, -3.0],
#        [2.0, 4.0, 10.8288, -3.0],
#        [2.0, 5.0, 16.9902, -3.0],
#        [2.0, 6.0, 3.82293, -6.0],
#        [2.0, 7.0, 11.0088, -6.0]]

# @elapsed predictor.predict(data)
pred_traj = predictor.predict(data)
print(pred_traj)
[-0.00252439 0.0052201;
-0.000815019 0.00596553;
-0.000815019 0.00596553;
-0.00225097 0.00542127;
-0.00239426 0.00534533;
-0.000815019 0.00596553;
-0.00240962 0.00533661]