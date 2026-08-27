# UniFlight Demo Plugin

A deliberately separate third-party distribution used to verify UniFlight Plugin API 1.0.

It registers six capabilities under the `demo.nereid:` namespace:

- `propulsion / constant-acceleration`
- `dynamics / point-mass-propulsion`
- `guard / time`
- `event_action / remove-vehicle`
- `output / specific-energy`
- `optimizer / grid-search`

Install after UniFlight:

```bash
python -m pip install -e . --no-build-isolation --no-deps
```

This plugin is reference/demo software, not a validated flight model.
