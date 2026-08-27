import numpy as np
from uniflight import core_3dof_schema, FrameGraph, Transform


def test_state_round_trip_and_immutability():
    s = core_3dof_schema()
    y = s.pack({"position": np.array([1.,2.,3.]), "velocity": np.array([4.,5.,6.]), "mass": 7.})
    out = s.unpack(y)
    np.testing.assert_allclose(out["position"], [1,2,3])
    assert out["mass"] == 7.0
    assert len(s.layout_hash) == 64


def test_012_frame_round_trip_near_machine_precision():
    th = 0.731
    c,sn = np.cos(th), np.sin(th)
    R = np.array([[c,-sn,0],[sn,c,0],[0,0,1.]])
    tf = Transform(R, np.array([4.,-2.,7.]), np.array([0.,0.,0.2]))
    fg = FrameGraph(); fg.add_transform("A","B",tf)
    v = np.array([2.3,-7.1,0.4])
    vb = fg.transform("A","B",0.0).vector(v)
    va = fg.transform("B","A",0.0).vector(vb)
    np.testing.assert_allclose(va, v, rtol=0, atol=2e-15)
    T = np.array([[2.,.3,0],[.3,5.,.2],[0,.2,1.]])
    Tb = fg.transform("A","B",0).tensor(T)
    Ta = fg.transform("B","A",0).tensor(Tb)
    np.testing.assert_allclose(Ta, T, rtol=0, atol=5e-15)
