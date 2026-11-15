#!/usr/bin/env bash
set -euo pipefail

program_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
config_dir=${XDG_CONFIG_HOME:-"$HOME/.config"}/microduck-voice
config_file=$config_dir/bailian.conf

fail() {
    printf 'Microduck voice: %s\n' "$*" >&2
    exit 1
}

read_credentials() {
    IFS= read -r realtime_url || fail "Expected the Realtime URL on the first line."
    IFS= read -r api_key || fail "Expected the API key on the second line."
    realtime_url=${realtime_url%$'\r'}
    api_key=${api_key%$'\r'}
}

validate_credentials() {
    [[ $realtime_url == wss://?* && $realtime_url != *[[:space:]]* ]] ||
        fail "Set a Realtime URL beginning with wss://."
    [[ -n $api_key && $api_key != *[[:space:]]* ]] ||
        fail "Set a nonempty Bailian API key."
}

usage='Usage: voice.sh [live|half-duplex|full-duplex|control|standup [--simulation]|standup-walk [--simulation]|probe-duplex|configure [--stdin]]'
mode=${1:-full-duplex}
case $mode in
    configure)
        if [[ $# == 2 && $2 == --stdin ]]; then
            read_credentials
        elif [[ $# == 1 && -t 0 ]]; then
            read -rp 'Bailian API Host (domain only): ' bailian_host
            [[ $bailian_host =~ ^[A-Za-z0-9][A-Za-z0-9.-]+$ ]] ||
                fail "Enter only the API Host domain."
            read -rsp 'Bailian API Key: ' api_key
            printf '\n' >&2
            realtime_url="wss://${bailian_host}/api-ws/v1/realtime?model=qwen3.8-omni-flash-realtime"
        else
            fail "Run configure in an interactive terminal, or pass two lines with configure --stdin."
        fi
        validate_credentials
        umask 077
        mkdir -p -- "$config_dir"
        chmod 700 -- "$config_dir"
        temporary_config=$(mktemp "$config_dir/.bailian.XXXXXX")
        trap 'rm -f -- "$temporary_config"' EXIT
        printf '%s\n' "$realtime_url" "$api_key" > "$temporary_config"
        mv -fT -- "$temporary_config" "$config_file"
        printf 'Saved board configuration to %s\n' "$config_file"
        ;;
    live|half-duplex|full-duplex|control|standup|standup-walk)
        simulation=()
        if [[ ( $mode == standup || $mode == standup-walk ) && $# == 2 && $2 == --simulation ]]; then
            simulation+=(--simulation)
        else
            [[ $# -le 1 ]] || fail "$usage"
        fi
        [[ -r $config_file ]] || fail "Run $program_dir/voice.sh configure once on the board."
        read_credentials < "$config_file"
        validate_credentials
        export DASHSCOPE_REALTIME_URL="$realtime_url"
        export DASHSCOPE_API_KEY="$api_key"
        if [[ $mode == half-duplex ]]; then
            exec "$program_dir/microduck-voice" --live
        fi
        if [[ $mode == control ]]; then
            candidate=$program_dir/control
            [[ -t 0 ]] || fail 'Use an interactive SSH terminal for action confirmation.'
            [[ -x $candidate/microduck-voice ]] || fail 'Voice control is missing.'
            exec "$candidate/microduck-voice" --live --full-duplex --control
        fi
        if [[ $mode == standup || $mode == standup-walk ]]; then
            candidate=$program_dir/standup
            if [[ $mode == standup-walk ]]; then
                simulation+=(--walk)
            fi
            [[ -x $candidate/microduck-voice ]] || fail 'Voice standup is missing.'
            runtime=${MICRODUCK_BOARD_RUNTIME:-/root/microduck-imu-update/walking-runtime}
            calibration=$runtime/calibration-native-home-restore30p24-20261106-230659
            umask 077
            private=$(mktemp -d /run/voice-standup.XXXXXX)
            export MICRODUCK_CONTROL_SOCKET=$private/control.sock
            env -u DASHSCOPE_API_KEY -u DASHSCOPE_REALTIME_URL python3 -B "$candidate/scripts/voice-standup.py" \
                --runtime "$runtime" --certificate "${MICRODUCK_IMU_CERTIFICATE:-$calibration/imu-direction-inherited.json}" \
                --reference "${MICRODUCK_STANDING_REFERENCE:-$calibration/native-home-reference.json}" \
                --socket "$MICRODUCK_CONTROL_SOCKET" "${simulation[@]}" &
            controller=$!
            voice_process=
            cleanup_standup() {
                if [[ -n $voice_process ]]; then
                    kill -TERM "$voice_process" 2>/dev/null || true
                    wait "$voice_process" || true
                fi
                kill -TERM "$controller" 2>/dev/null || true
                wait "$controller" || true
                rm -f -- "$private/control.sock.lock"
                rmdir -- "$private"
            }
            trap cleanup_standup EXIT
            trap 'exit 130' INT HUP TERM
            for _ in {1..40}; do
                [[ -S $MICRODUCK_CONTROL_SOCKET ]] && break
                kill -0 "$controller" 2>/dev/null || fail 'Standup bridge failed to start.'
                sleep .05
            done
            [[ -S $MICRODUCK_CONTROL_SOCKET ]] || fail 'Standup bridge is unavailable.'
            "$candidate/microduck-voice" --live --full-duplex "--$mode" <&0 &
            voice_process=$!
            wait "$voice_process"
            exit $?
        fi
        [[ -x $program_dir/microduck-voice-duplex ]] || fail "Full-duplex client is missing. Use half-duplex to fall back."
        exec "$program_dir/microduck-voice-duplex" --live --full-duplex
        ;;
    probe-duplex)
        [[ $# -eq 1 ]] || fail "$usage"
        [[ -x $program_dir/microduck-voice-duplex ]] || fail "Full-duplex client is missing."
        exec "$program_dir/microduck-voice-duplex" --probe-duplex
        ;;
    -h|--help)
        printf '%s\n' "$usage" \
            'configure saves the API Host and key on the board.' \
            'configure --stdin reads the WSS URL and key on separate lines.' \
            'Without an argument, live and full-duplex start full-duplex dialogue.' \
            'half-duplex waits for each reply to finish before listening again.' \
            'control starts full-duplex with terminal action confirmation.' \
            'standup starts floor recovery after a one-second delay; --simulation exercises it without motors.' \
            'standup-walk also accepts a separate spoken command to move forward in the same session.' \
            'probe-duplex runs a 60-second local audio test without connecting to Bailian.'
        ;;
    *)
        fail "$usage"
        ;;
esac
