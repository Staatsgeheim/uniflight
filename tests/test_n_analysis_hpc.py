from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import numpy as np

from uniflight.analysis import (
    AnalysisCase, MissionCampaignRunner, MissionMonteCarlo, MonteCarloVariable,
    OptimizationBatch, OptimizationStart, ParameterSweep, SobolSensitivity,
    SobolVariable, SweepVariable, mission_case_worker,
)
from uniflight.hpc import ExternalExecutorBackend, ProcessBackend, SerialBackend
from uniflight.mission import load_mission
from uniflight.montecarlo import NormalDispersion, UniformDispersion
from uniflight.result_store import SQLiteResultStore, StoredCase

ROOT=Path(__file__).resolve().parents[1]
MISSION=ROOT/'missions'/'nereid_n_analysis.yaml'


def _square(x):
    return x*x


def test_sweep_cartesian_and_stable_case_ids():
    sweep=ParameterSweep([
        SweepVariable('a','/a',(1.0,2.0)), SweepVariable('b','/b',(3.0,4.0,5.0))
    ])
    a=sweep.cases(); b=sweep.cases()
    assert len(a)==6
    assert [x.case_id for x in a]==[x.case_id for x in b]
    assert a[0].overrides=={'/a':1.0,'/b':3.0}


def test_monte_carlo_is_seed_deterministic():
    vars=[MonteCarloVariable('x','/x',NormalDispersion(0,1)),
          MonteCarloVariable('y','/y',UniformDispersion(-1,1))]
    a=MissionMonteCarlo(vars,cases=8,seed=42).cases()
    b=MissionMonteCarlo(vars,cases=8,seed=42).cases()
    assert [x.parameters for x in a]==[x.parameters for x in b]
    assert [x.case_id for x in a]==[x.case_id for x in b]


def test_sobol_linear_known_indices():
    study=SobolSensitivity([
        SobolVariable('x','/x',0,1), SobolVariable('y','/y',0,1)
    ],base_samples=1024,seed=3)
    rows=[]
    for c in study.cases():
        value=float(c.parameters['x'])+0.5*float(c.parameters['y'])
        rows.append(StoredCase(c.case_id,c.index,c.kind,'completed',c.parameters,{'metric':value},None,0.0))
    result=study.analyze(rows,'metric')
    assert np.allclose(result.first_order,[0.8,0.2],atol=0.015)
    assert np.allclose(result.total_order,[0.8,0.2],atol=0.015)


def test_result_store_is_transactional_checkpoint(tmp_path):
    path=tmp_path/'analysis.sqlite'
    with SQLiteResultStore(path) as store:
        store.begin_campaign('c','sweep',mission_sha256='abc',metadata={'x':1})
        store.write_case('c',StoredCase('one',0,'sweep','completed',{'x':1},{'y':2},None,0.1))
        assert store.completed_case_ids('c')=={'one'}
        assert store.summary('c')['case_counts']=={'completed':1}
    with SQLiteResultStore(path) as store:
        assert store.completed_case_ids('c')=={'one'}
        out=store.export_json('c',tmp_path/'export.json')
        payload=json.loads(out.read_text())
        assert payload['cases'][0]['metrics']['y']==2


def test_external_executor_backend():
    with ThreadPoolExecutor(max_workers=2) as ex:
        backend=ExternalExecutorBackend(ex,workers=2)
        assert list(backend.map(_square,[1,2,3]))==[1,4,9]
        assert backend.workers==2


def test_process_backend_uses_multiple_workers():
    backend=ProcessBackend(max_workers=2,chunksize=1)
    assert list(backend.map(_square,[1,2,3,4]))==[1,4,9,16]


def test_mission_analysis_declaration_validates():
    doc=load_mission(MISSION)
    analysis=doc.raw['analysis']
    assert analysis['sweeps'][0]['id']=='propulsion-grid'
    assert analysis['sobol'][0]['metric']=='final_altitude'


def test_campaign_resume_executes_only_missing_cases(tmp_path):
    cases=ParameterSweep([SweepVariable('mass_flow','/vehicles/vehicle/phases/0/dynamics/ideal_rocket/mass_flow',(0.4,0.5))]).cases()
    store=SQLiteResultStore(tmp_path/'campaign.sqlite')
    try:
        runner=MissionCampaignRunner(MISSION,backend=SerialBackend(),store=store)
        first=runner.run_cases(cases,campaign_id='resume',kind='sweep')
        second=runner.run_cases(cases,campaign_id='resume',kind='sweep')
        assert first.executed_cases==2 and first.resumed_cases==0
        assert second.executed_cases==0 and second.resumed_cases==2
        assert len(store.cases('resume'))==2
    finally:
        store.close()


def test_optimization_batch_resolves_design_variable_initial_pointer():
    doc=load_mission(MISSION)
    batch=OptimizationBatch([
        OptimizationStart('low',{'mass_flow':0.3}), OptimizationStart('high',{'mass_flow':0.9})
    ])
    cases=batch.cases(doc)
    assert cases[0].overrides['/optimization/design_variables/0/initial']==0.3
    assert cases[1].parameters['start']=='high'


def test_process_backend_rejects_unpickleable_worker():
    import pytest
    backend=ProcessBackend(max_workers=2)
    with pytest.raises(TypeError):
        list(backend.map(lambda x:x,[1,2]))


def test_duplicate_analysis_ids_are_rejected():
    import pytest
    from uniflight.mission import MissionDocument, MissionValidationError
    doc=load_mission(MISSION)
    raw=doc.mutable_copy()
    raw['analysis']['sobol'][0]['id']=raw['analysis']['sweeps'][0]['id']
    with pytest.raises(MissionValidationError):
        MissionDocument(raw,base_directory=doc.base_directory)


def test_failed_case_is_recorded_not_raised(tmp_path):
    # Deliberately create an invalid physical override that compiles but fails
    # during model construction/execution.
    case=AnalysisCase(0,'sweep',{'/vehicles/vehicle/phases/0/dynamics/ideal_rocket/mass_flow':-1.0},{'bad':-1.0})
    store=SQLiteResultStore(tmp_path/'fail.sqlite')
    try:
        ex=MissionCampaignRunner(MISSION,backend=SerialBackend(),store=store).run_cases([case],campaign_id='fail',kind='sweep')
        rows=store.cases('fail')
        assert ex.failed_cases==1 and rows[0].status=='failed'
        assert rows[0].metrics['success'] is False
        assert rows[0].error
    finally:
        store.close()


def test_analysis_cli_list(capsys):
    from uniflight.analysis_cli import main
    assert main(['list',str(MISSION)])==0
    payload=json.loads(capsys.readouterr().out)
    assert payload['monte_carlo'] is True
    assert 'propulsion-grid' in payload['sweeps']
