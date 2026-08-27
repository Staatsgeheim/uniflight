import numpy as np
from uniflight.verification import TolerancePolicy, RegressionBaseline, ReferenceTimeHistory, compare_time_histories, observed_order
from uniflight.verification_cases import run_builtin_verification


def test_tolerance_policy():
    p=TolerancePolicy(absolute=1.0,relative=.1,scale_floor=2.0)
    assert p.allowed(5.0)==1.5 and p.accepts(1.49,5.0) and not p.accepts(1.51,5.0)


def test_observed_order():
    h=np.array([.2,.1,.05,.025]); e=3*h**4
    assert abs(observed_order(h,e)-4.0)<1e-12


def test_regression_baseline_pass_and_missing():
    b=RegressionBaseline("b",{"x":10.,"y":2.},{"x":TolerancePolicy(absolute=.1),"y":TolerancePolicy(absolute=.1)})
    r=b.compare({"x":10.05})
    assert r[0].passed and r[1].status=="FAIL"


def test_reference_time_history_validation():
    r=ReferenceTimeHistory(np.array([0.,1.,2.]),{"x":np.array([1.,2.,3.])})
    assert r.time.size==3


def test_time_history_interpolation_passes():
    ref=ReferenceTimeHistory(np.array([0.,1.,2.]),{"x":np.array([0.,1.,2.])})
    act=ReferenceTimeHistory(np.array([0.,.5,1.,1.5,2.]),{"x":np.array([0.,.5,1.,1.5,2.])})
    result=compare_time_histories(ref,act,tolerance=TolerancePolicy(absolute=1e-12))
    assert result[0].passed


def test_time_history_missing_channel_fails():
    ref=ReferenceTimeHistory(np.array([0.,1.]),{"x":np.array([0.,1.])})
    act=ReferenceTimeHistory(np.array([0.,1.]),{"y":np.array([0.,1.])})
    assert compare_time_histories(ref,act,channels=["x"])[0].status=="FAIL"


def test_builtin_suite_has_no_internal_failures():
    r=run_builtin_verification()
    assert r.failed==0
    assert r.passed==12
    assert r.skipped==2


def test_builtin_suite_case_ids_unique():
    r=run_builtin_verification(); ids=[x.case_id for x in r.results]
    assert len(ids)==len(set(ids))


def test_report_serializable(tmp_path):
    r=run_builtin_verification(); p=r.write_json(tmp_path/"r.json"); m=r.write_markdown(tmp_path/"r.md")
    assert p.exists() and m.exists()


def test_external_cases_not_counted_as_passes():
    r=run_builtin_verification(); ext=[x for x in r.results if x.case_id.startswith("NESC")]
    assert len(ext)==2 and all(x.skipped for x in ext)
