# AGENTS.md — `ec`

Shared rules for the whole system live in **`~/grow/AGENTS.md`** (the
`deployment` repo): workflow (issue → branch → PR), sourcing, build,
conventions, traps. Read that first. This file is only what is specific to
this repo.

## Git identity

Commit here as **`ec-agent`** so history shows which project's agent did
what. Once per clone (the setting is repo-local, not versioned):

```bash
git config user.name  "ec-agent"
git config user.email "ec-agent@grow.local"
```

Check with `git config user.name` before committing. Humans committing by
hand override with `git -c user.name=… -c user.email=… commit`.

## Specific

- Env: `I2C_ADDRESS` (100), `METRIC_INTERVAL` (10), `READ_CMD` (`R`),
  `NODE_NAME` (`metric`), `WINDOW_SIZE` (5). Publishes `Float32` on the
  node's name topic.
- Deployed **three times** by the `metric` role (ec/ph/temp, different
  I2C addresses) — one package, three compose services.
