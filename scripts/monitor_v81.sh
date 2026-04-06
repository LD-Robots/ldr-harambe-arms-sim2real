#!/bin/bash
# V8.1 auto-monitor: metrics every 5 min, play check every 5000 iters
# Run: bash scripts/monitor_v81.sh &

LOG_FILE="$HOME/Git/ldr-harambe-arms-sim2real/logs/v81_monitor.log"
mkdir -p "$(dirname "$LOG_FILE")"

LAST_PLAY_CHECK=0
PLAY_INTERVAL=5000

while true; do
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    # Check if training running
    RUNNING=$(ps aux | grep "train.py.*Harambe" | grep -v grep | wc -l)
    if [ "$RUNNING" -eq 0 ]; then
        echo "[$TIMESTAMP] TRAINING STOPPED!" >> "$LOG_FILE"
        echo "[$TIMESTAMP] TRAINING STOPPED!"
        break
    fi

    # Find the latest run dir (v8.1 = newest)
    RUN_DIR=$(ls -td ~/IsaacLab/logs/rsl_rl/harambe_flat/2026-04-01_* 2>/dev/null | head -1)
    if [ -z "$RUN_DIR" ]; then
        echo "[$TIMESTAMP] No run dir found yet" >> "$LOG_FILE"
        sleep 60
        continue
    fi

    # Get metrics
    METRICS=$(cd ~/IsaacLab && conda run -n env_isaaclab python -c "
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
ea = EventAccumulator('$RUN_DIR/')
ea.Reload()
tags = ['Train/mean_reward', 'Train/mean_episode_length', 'Episode_Reward/termination_penalty', 'Episode_Reward/dof_torques_l2', 'Curriculum/domain_rand/difficulty', 'Curriculum/domain_rand/reward_ema']
parts = []
step = 0
for tag in tags:
    events = ea.Scalars(tag)
    if events:
        short = tag.split('/')[-1]
        parts.append(f'{short}={events[-1].value:.4f}')
        if tag == 'Train/mean_reward':
            step = events[-1].step
parts.insert(0, f'iter={step}')
print(' '.join(parts))
" 2>/dev/null)

    echo "[$TIMESTAMP] $METRICS" >> "$LOG_FILE"
    echo "[$TIMESTAMP] $METRICS"

    # Extract current iteration
    CURRENT_ITER=$(echo "$METRICS" | grep -oP 'iter=\K[0-9]+')

    # Play check every 5000 iters
    if [ -n "$CURRENT_ITER" ] && [ "$CURRENT_ITER" -ge $((LAST_PLAY_CHECK + PLAY_INTERVAL)) ]; then
        echo "[$TIMESTAMP] === PLAY CHECK at iter $CURRENT_ITER ===" >> "$LOG_FILE"
        echo "[$TIMESTAMP] === PLAY CHECK at iter $CURRENT_ITER ==="

        # Find latest checkpoint
        CKPT=$(ls -t "$RUN_DIR"/model_*.pt 2>/dev/null | head -1)
        if [ -n "$CKPT" ]; then
            CKPT_NAME=$(basename "$CKPT")
            echo "[$TIMESTAMP] Using checkpoint: $CKPT_NAME" >> "$LOG_FILE"

            # Pause training
            TRAIN_PID=$(ps aux | grep "train.py.*Harambe" | grep -v grep | awk '{print $2}')
            kill -STOP $TRAIN_PID 2>/dev/null

            # Run play headless for 200 steps, check mean episode length
            PLAY_RESULT=$(cd ~/IsaacLab && timeout 120 conda run -n env_isaaclab python -c "
import torch
import numpy as np

ckpt = torch.load('$CKPT', map_location='cpu', weights_only=False)
model = ckpt['actor_state_dict']

# Check for NaN/Inf
has_nan = any(torch.isnan(v).any().item() for v in model.values() if hasattr(v, 'shape'))
has_inf = any(torch.isinf(v).any().item() for v in model.values() if hasattr(v, 'shape'))

# Check action output magnitude with zero obs
import torch.nn as nn
class Actor(nn.Module):
    def __init__(self):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(87, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, 128), nn.ELU(),
            nn.Linear(128, 25),
        )
    def forward(self, x):
        return self.mlp(x)

actor = Actor()
mlp_state = {k.replace('mlp.', ''): v for k, v in model.items() if k.startswith('mlp.')}
actor.mlp.load_state_dict(mlp_state)
actor.eval()

# Test with zero obs and random obs
with torch.no_grad():
    zero_out = actor(torch.zeros(1, 87))
    rand_out = actor(torch.randn(100, 87))

print(f'nan={has_nan} inf={has_inf}')
print(f'zero_obs_actions: min={zero_out.min():.3f} max={zero_out.max():.3f} mean={zero_out.mean():.3f}')
print(f'rand_obs_actions: min={rand_out.min():.3f} max={rand_out.max():.3f} std={rand_out.std():.3f}')
print(f'action_scale=0.10 -> actual_range: {rand_out.min()*0.1:.4f} to {rand_out.max()*0.1:.4f} rad')
" 2>/dev/null)

            echo "[$TIMESTAMP] PLAY: $PLAY_RESULT" >> "$LOG_FILE"
            echo "[$TIMESTAMP] PLAY: $PLAY_RESULT"

            # Resume training
            kill -CONT $TRAIN_PID 2>/dev/null
            echo "[$TIMESTAMP] Training resumed" >> "$LOG_FILE"
        fi

        LAST_PLAY_CHECK=$CURRENT_ITER
    fi

    sleep 300  # 5 min
done
