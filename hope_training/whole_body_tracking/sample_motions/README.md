# Sample Motions

Scratch directory for your own motion clips (`.npz`).

The shipped reference clips live at [`hope_training/motions/preprocessed/`](../../motions/preprocessed/)
(`hope_forehand` / `hope_backhand`, each an `.npz` with a `.yaml` sidecar). They are
physically-neutral placeholders: replace them with real retargeted swings before training a policy
you intend to deploy. The expected `.npz` arrays and `.yaml` metadata are documented in
[`docs/REPLACE_MOTIONS.md`](../../../docs/REPLACE_MOTIONS.md).

Point training at clips in this directory (clip 0 = forehand, clip 1 = backhand):

```bash
python scripts/train.py task=HOPEPingPong algo=ppo headless=true \
    motion_file=sample_motions/my_forehand.npz \
    motion_file_2=sample_motions/my_backhand.npz
```
