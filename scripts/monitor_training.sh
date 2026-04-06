#!/bin/bash
# Auto-monitor v8 training every 5 minutes
# Run: bash scripts/monitor_training.sh &

RUN_DIR="$HOME/IsaacLab/logs/rsl_rl/harambe_flat/2026-03-31_16-07-30"
LOG_FILE="$HOME/Git/ldr-harambe-arms-sim2real/logs/v8_monitor.log"

while true; do
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

    # Check if training running
    RUNNING=$(ps aux | grep "train.py.*Harambe" | grep -v grep | wc -l)

    if [ "$RUNNING" -eq 0 ]; then
        echo "[$TIMESTAMP] TRAINING STOPPED!" >> "$LOG_FILE"
        echo "[$TIMESTAMP] TRAINING STOPPED!"
        break
    fi

    # Get metrics
    METRICS=$(cd ~/IsaacLab && conda run -n env_isaaclab python -c "
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
ea = EventAccumulator('$RUN_DIR/')
ea.Reload()
r = ea.Scalars('Train/mean_reward')
e = ea.Scalars('Train/mean_episode_length')
t = ea.Scalars('Episode_Reward/termination_penalty')
d = ea.Scalars('Curriculum/domain_rand/difficulty')
if r and e:
    print(f'iter={r[-1].step} reward={r[-1].value:.2f} ep_len={e[-1].value:.1f} term={t[-1].value:.3f} diff={d[-1].value:.3f}')
" 2>/dev/null)

    echo "[$TIMESTAMP] $METRICS" >> "$LOG_FILE"
    echo "[$TIMESTAMP] $METRICS"

    sleep 300  # 5 minutes
done
