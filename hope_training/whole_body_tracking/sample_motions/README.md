# Sample Motions

Generated `.npz` files in this directory are ignored by git.

Create the smoke clip with:

```bash
hope_isaac_py scripts/create_smoke_motion.py \
  --headless \
  --frames 120 \
  --output sample_motions/agibot_a3_smoke_stand.npz
```

The smoke clip is only for pipeline verification. Replace it with a retargeted
ping-pong motion before running meaningful training.
