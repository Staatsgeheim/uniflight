# Milestone N full-scale validation

The sandbox reference exercises all N functionality but is not intended as a throughput ceiling. On a 32-logical-core workstation, use the commands below after installing UniFlight 0.14.0.

## Process hygiene

PowerShell:

```powershell
$env:OPENBLAS_NUM_THREADS="1"
$env:OMP_NUM_THREADS="1"
$env:MKL_NUM_THREADS="1"
```

Linux/macOS:

```bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

## Monte Carlo

The reference mission declares 32 cases, but CLI count may be overridden:

```bash
uniflight-analysis monte-carlo missions/nereid_n_analysis.yaml \
  --cases 10000 --backend process --workers 31 \
  --campaign-id nereid-n-analysis.mc10k \
  --store reports/n_full.sqlite \
  --export reports/n_mc10k.json
```

Rerunning the exact command resumes the ledger instead of recomputing completed cases.

## Sobol

For production sensitivity work increase `analysis.sobol[].base_samples` to 512 or 1024. With two uncertain parameters, `N=1024` requires 4096 trajectories.

```bash
uniflight-analysis sobol missions/nereid_n_analysis.yaml propulsion-sensitivity \
  --backend process --workers 31 --store reports/n_full.sqlite \
  --export reports/n_sobol.json
```

## Multistart optimization

Add additional starts to `analysis.optimization_batches[].starts`, then:

```bash
uniflight-analysis optimize-batch missions/nereid_n_analysis.yaml multistart \
  --backend process --workers 31 --store reports/n_full.sqlite \
  --export reports/n_optbatch.json
```

## Worker-count benchmark

For long campaigns benchmark physical cores and SMT configurations rather than assuming `logical_cpus - 1` is fastest. Compare, for example:

```text
workers = 8, 12, 16, 20, 24, 28, 31
```

Thermal/power limits, BLAS oversubscription, memory bandwidth, and the cost of each trajectory determine the optimum.

## Result handoff

The SQLite file is the authoritative checkpoint. JSON exports are intended for portable review and can be returned for statistical analysis.
