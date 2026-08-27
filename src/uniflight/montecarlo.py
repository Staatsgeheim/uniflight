from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Mapping
import numpy as np


class Dispersion:
    def sample(self,rng:np.random.Generator): raise NotImplementedError

@dataclass(frozen=True,slots=True)
class NormalDispersion(Dispersion):
    mean: float
    std: float
    def __post_init__(self):
        if not np.isfinite(self.mean) or not np.isfinite(self.std) or self.std<0: raise ValueError("invalid normal dispersion")
    def sample(self,rng): return float(rng.normal(self.mean,self.std))

@dataclass(frozen=True,slots=True)
class UniformDispersion(Dispersion):
    low: float
    high: float
    def __post_init__(self):
        if not np.isfinite(self.low) or not np.isfinite(self.high) or self.high<self.low: raise ValueError("invalid uniform dispersion")
    def sample(self,rng): return float(rng.uniform(self.low,self.high))

@dataclass(frozen=True,slots=True)
class MonteCarloCaseResult:
    index:int
    seed:int
    parameters:Mapping[str,float]
    metrics:Mapping[str,float|bool]

@dataclass(frozen=True,slots=True)
class MetricStatistics:
    mean:float
    std:float
    minimum:float
    maximum:float
    p05:float
    median:float
    p95:float

@dataclass(frozen=True,slots=True)
class MonteCarloSummary:
    cases:tuple[MonteCarloCaseResult,...]
    success_rate:float
    statistics:Mapping[str,MetricStatistics]

class MonteCarloRunner:
    """Deterministic serial Monte Carlo runner using NumPy SeedSequence streams."""
    def __init__(self,case_function:Callable[[Mapping[str,float],np.random.Generator],Mapping[str,float|bool]],
                 dispersions:Mapping[str,Dispersion],base_seed:int=0):
        self.case_function=case_function; self.dispersions=dict(dispersions); self.base_seed=int(base_seed)

    def run(self,n_cases:int)->MonteCarloSummary:
        if n_cases<=0: raise ValueError("n_cases must be positive")
        children=np.random.SeedSequence(self.base_seed).spawn(n_cases)
        results=[]
        for i,ss in enumerate(children):
            rng=np.random.default_rng(ss)
            params={k:d.sample(rng) for k,d in self.dispersions.items()}
            metrics=dict(self.case_function(params,rng))
            seed=int(ss.generate_state(1,dtype=np.uint64)[0])
            results.append(MonteCarloCaseResult(i,seed,params,metrics))
        successes=np.array([bool(r.metrics.get("success",True)) for r in results],dtype=bool)
        names=sorted({k for r in results for k,v in r.metrics.items() if k!="success" and isinstance(v,(int,float,np.integer,np.floating))})
        stats={}
        for name in names:
            vals=np.array([float(r.metrics[name]) for r in results if name in r.metrics and np.isfinite(float(r.metrics[name]))],dtype=float)
            if vals.size:
                stats[name]=MetricStatistics(float(vals.mean()),float(vals.std(ddof=0)),float(vals.min()),float(vals.max()),
                                             float(np.quantile(vals,.05)),float(np.median(vals)),float(np.quantile(vals,.95)))
        return MonteCarloSummary(tuple(results),float(successes.mean()),stats)
