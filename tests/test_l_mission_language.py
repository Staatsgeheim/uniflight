from __future__ import annotations
from copy import deepcopy
from pathlib import Path
import json
import numpy as np
import pytest

from uniflight import (
    MissionDocument, MissionValidationError, MissionCompilationError,
    MissionCompiler, load_mission, mission_json_schema, mission_sha256,
    pointer_get, pointer_set,
)
from uniflight.mission_cli import main as cli_main

ROOT=Path(__file__).resolve().parents[1]
YAML=ROOT/'missions'/'nereid_l.yaml'
TOML=ROOT/'missions'/'nereid_l_minimal.toml'


def test_yaml_load_and_digest_is_deterministic():
    a=load_mission(YAML); b=load_mission(YAML)
    assert a.mission_id=='nereid-l-declarative'
    assert a.digest_sha256==b.digest_sha256==mission_sha256(a.raw)
    assert len(a.digest_sha256)==64


def test_toml_load_compile_and_run():
    doc=load_mission(TOML)
    rep=MissionCompiler().compile(doc).run()
    assert rep.success
    assert 990.0 < rep.outputs['altitude'] < 1010.0


def test_json_pointer_get_set_and_immutable_override():
    doc=load_mission(YAML)
    ptr='/vehicles/lander/phases/0/dynamics/ideal_rocket/mass_flow'
    assert pointer_get(doc.raw,ptr)==1.0
    raw=doc.mutable_copy(); pointer_set(raw,ptr,1.2)
    assert pointer_get(raw,ptr)==1.2
    assert pointer_get(doc.raw,ptr)==1.0
    changed=doc.with_overrides({ptr:0.8})
    assert pointer_get(changed.raw,ptr)==0.8
    assert changed.digest_sha256 != doc.digest_sha256


def test_unknown_core_key_rejected():
    raw=load_mission(TOML).mutable_copy()
    raw['mission']['typo_solver']=1
    with pytest.raises(MissionValidationError,match='unknown key'):
        MissionDocument(raw)


def test_cross_reference_rejected():
    raw=load_mission(TOML).mutable_copy()
    raw['vehicles']['probe']['body']='missing'
    with pytest.raises(MissionValidationError,match='unknown body'):
        MissionDocument(raw)


def test_dataset_is_exact_version_pinned_and_checksummed():
    comp=MissionCompiler().compile(load_mission(YAML))
    inv=comp.data_catalog.inventory()
    assert len(inv)==1
    assert inv[0][0:2]==('nereid-k.atmosphere','1.0')
    assert len(inv[0][2])==64


def test_dataset_declaration_mismatch_fails_compilation():
    raw=load_mission(YAML).mutable_copy()
    raw['datasets'][0]['version']='9.9'
    doc=MissionDocument(raw,base_directory=YAML.parent)
    with pytest.raises(MissionCompilationError,match='does not match file provenance'):
        MissionCompiler().compile(doc)


def test_yaml_reference_executes_phase_and_dof_transitions():
    rep=MissionCompiler().compile(load_mission(YAML)).run()
    assert rep.success
    assert [round(e['time'],9) for e in rep.events]==[5.0,8.0]
    assert [e['note'] for e in rep.events]==['phase -> coast-6dof','phase -> coast-3dof']
    assert rep.final_vehicles['lander']['dof']==3
    assert rep.final_vehicles['lander']['mode']=='coast-3dof'
    assert rep.outputs['final_mass']==pytest.approx(95.0,abs=1e-9)
    assert rep.outputs['final_altitude']==pytest.approx(961.5848180233734,rel=2e-10)


def test_output_metrics_are_runtime_derived():
    rep=MissionCompiler().compile(load_mission(YAML)).run()
    assert rep.outputs['final_time']==12.0
    assert rep.outputs['vehicle_count']==1.0
    assert rep.outputs['final_speed'] > 100.0
    assert rep.outputs['final_altitude'] > 900.0


def test_optimization_declaration_builds_h_problem_and_solves():
    compiler=MissionCompiler(); doc=load_mission(YAML)
    result=compiler.optimize(doc)
    assert result.success
    assert 0.5 <= result.design['mass_flow'] <= 1.5
    assert result.metrics['final_altitude'] >= 600.0-1e-3
    assert result.metrics['final_mass'] > 96.0
    assert result.max_constraint_violation <= 1e-7


def test_monte_carlo_sampling_is_seed_deterministic():
    compiler=MissionCompiler(); doc=load_mission(YAML)
    a=compiler.sample_monte_carlo(doc,cases=5,seed=42)
    b=compiler.sample_monte_carlo(doc,cases=5,seed=42)
    av=[x['values']['mass_flow_dispersion'] for x in a]
    bv=[x['values']['mass_flow_dispersion'] for x in b]
    assert av==bv
    assert len(set(av))==5


def test_monte_carlo_pointer_reference_is_validated():
    raw=load_mission(YAML).mutable_copy()
    raw['monte_carlo']['dispersions'][0]['pointer']='/no/such/value'
    with pytest.raises((MissionValidationError,KeyError)):
        MissionDocument(raw,base_directory=YAML.parent)


def test_registry_is_strict_and_extensible():
    from uniflight import MissionRegistry
    r=MissionRegistry()
    r.register('thing','constant',lambda spec,ctx:(spec['x'],ctx['y']))
    assert r.build('thing','constant',{'x':2},{'y':3})==(2,3)
    with pytest.raises(KeyError): r.register('thing','constant',lambda s,c:None)
    with pytest.raises(MissionCompilationError): r.build('thing','missing',{}, {})


def test_schema_contract_exposes_version_and_required_sections():
    schema=mission_json_schema()
    assert schema['properties']['format_version']['const']=='1.0'
    assert set(['format_version','mission','bodies','vehicles']) <= set(schema['required'])
    assert schema['additionalProperties'] is False


def test_cli_validate_inspect_run_and_schema(tmp_path,capsys):
    assert cli_main(['validate',str(TOML)])==0
    assert 'VALID nereid-l-toml' in capsys.readouterr().out
    assert cli_main(['inspect',str(TOML)])==0
    inspect=json.loads(capsys.readouterr().out)
    assert inspect['vehicles']==['probe']
    out=tmp_path/'report.json'
    assert cli_main(['run',str(TOML),'--output',str(out)])==0
    capsys.readouterr()
    report=json.loads(out.read_text())
    assert report['metadata']['mission_id']=='nereid-l-toml'
    schema_out=tmp_path/'schema.json'
    assert cli_main(['schema','--output',str(schema_out)])==0
    assert json.loads(schema_out.read_text())['title'].startswith('UniFlight')


def test_declarative_rigid_staging_spawns_two_vehicles():
    doc=load_mission(ROOT/'missions'/'nereid_l_staging.yaml')
    rep=MissionCompiler().compile(doc).run()
    assert rep.success
    assert set(rep.final_vehicles)=={'upper','booster'}
    assert rep.outputs['active_vehicles']==2.0
    assert len(rep.events)==1
    assert rep.events[0]['event']=='stage-separation'
    assert rep.events[0]['time']==pytest.approx(2.0,abs=1e-10)
    assert rep.events[0]['note']=='declarative two-body staging'


def test_json_format_loads_and_runs(tmp_path):
    raw=load_mission(TOML).mutable_copy()
    path=tmp_path/'mission.json'
    path.write_text(json.dumps(raw),encoding='utf-8')
    rep=MissionCompiler().compile(load_mission(path)).run()
    assert rep.success
    assert rep.outputs['altitude'] > 990.0
