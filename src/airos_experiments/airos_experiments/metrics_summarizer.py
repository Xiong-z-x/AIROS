from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from statistics import mean, pstdev
from typing import Any


def _valid_numbers(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if isinstance(value, int | float):
            values.append(float(value))
    return values


def load_rows(path: Path) -> list[dict[str, Any]]:
    rows = [
        json.loads(line)
        for line in path.read_text(encoding='utf-8').splitlines()
        if line.strip()
    ]
    if not rows:
        raise RuntimeError(f'no trial rows in {path}')
    return rows


def summarize(path: Path) -> dict[str, Any]:
    rows = load_rows(path)

    success_count = sum(1 for row in rows if row.get('success') is True)
    elapsed = _valid_numbers(rows, 'elapsed_sec')
    path_lengths = _valid_numbers(rows, 'path_length_m')
    stops = _valid_numbers(rows, 'emergency_stop_count')
    collisions = _valid_numbers(rows, 'collision_count')
    min_distances = _valid_numbers(rows, 'minimum_obstacle_distance_m')
    mean_cmd_periods = _valid_numbers(rows, 'mean_cmd_period_sec')
    max_cmd_periods = _valid_numbers(rows, 'max_cmd_period_sec')

    return {
        'trial_count': len(rows),
        'success_count': success_count,
        'success_rate': round(success_count / len(rows), 3),
        'mean_elapsed_sec': round(mean(elapsed), 3) if elapsed else None,
        'mean_path_length_m': (
            round(mean(path_lengths), 3) if path_lengths else None
        ),
        'total_emergency_stop_count': int(sum(stops)) if stops else 0,
        'collision_count': int(sum(collisions)) if collisions else None,
        'minimum_obstacle_distance_m': (
            min(min_distances) if min_distances else None
        ),
        'mean_cmd_period_sec': (
            round(mean(mean_cmd_periods), 4) if mean_cmd_periods else None
        ),
        'max_cmd_period_sec': (
            round(max(max_cmd_periods), 4) if max_cmd_periods else None
        ),
    }


def _mission_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    mission_ids = sorted(
        {
            str(row.get('mission_id'))
            for row in rows
            if row.get('mission_id') is not None
        }
    )
    summaries: list[dict[str, Any]] = []
    for mission_id in mission_ids:
        mission_rows = [
            row for row in rows if str(row.get('mission_id')) == mission_id
        ]
        success_count = sum(
            1 for row in mission_rows if row.get('success') is True
        )
        elapsed = _valid_numbers(mission_rows, 'elapsed_sec')
        path_lengths = _valid_numbers(mission_rows, 'path_length_m')
        min_distances = _valid_numbers(
            mission_rows,
            'minimum_obstacle_distance_m',
        )
        max_cmd = _valid_numbers(mission_rows, 'max_cmd_period_sec')
        summaries.append(
            {
                'mission_id': mission_id,
                'route_id': mission_rows[0].get('route_id'),
                'trial_count': len(mission_rows),
                'success_count': success_count,
                'success_rate': round(success_count / len(mission_rows), 3),
                'mean_elapsed_sec': (
                    round(mean(elapsed), 3) if elapsed else None
                ),
                'std_elapsed_sec': (
                    round(pstdev(elapsed), 3) if len(elapsed) > 1 else 0.0
                ),
                'mean_path_length_m': (
                    round(mean(path_lengths), 3) if path_lengths else None
                ),
                'minimum_obstacle_distance_m': (
                    round(min(min_distances), 3) if min_distances else None
                ),
                'max_cmd_period_sec': (
                    round(max(max_cmd), 4) if max_cmd else None
                ),
            }
        )
    return summaries


def _format_value(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, float):
        return f'{value:g}'
    return str(value)


def write_csv(rows: list[dict[str, Any]], path: Path) -> None:
    mission_rows = _mission_summary(rows)
    fieldnames = [
        'mission_id',
        'route_id',
        'trial_count',
        'success_count',
        'success_rate',
        'mean_elapsed_sec',
        'std_elapsed_sec',
        'mean_path_length_m',
        'minimum_obstacle_distance_m',
        'max_cmd_period_sec',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(mission_rows)


def write_markdown(
    summary: dict[str, Any],
    rows: list[dict[str, Any]],
    path: Path,
    source_path: Path,
) -> None:
    mission_rows = _mission_summary(rows)
    headers = [
        'mission_id',
        'trials',
        'success',
        'mean_elapsed_s',
        'mean_path_m',
        'min_scan_m',
        'max_cmd_period_s',
    ]
    table = [
        '| ' + ' | '.join(headers) + ' |',
        '| ' + ' | '.join(['---'] * len(headers)) + ' |',
    ]
    for row in mission_rows:
        table.append(
            '| '
            + ' | '.join([
                _format_value(row['mission_id']),
                _format_value(row['trial_count']),
                f"{row['success_count']}/{row['trial_count']}",
                _format_value(row['mean_elapsed_sec']),
                _format_value(row['mean_path_length_m']),
                _format_value(row['minimum_obstacle_distance_m']),
                _format_value(row['max_cmd_period_sec']),
            ])
            + ' |'
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        '\n'.join([
            '# Single Floor Lab Navigation Summary',
            '',
            f'Source: `{source_path.as_posix()}`',
            '',
            '## Aggregate Metrics',
            '',
            f"- Trials: {summary['trial_count']}",
            f"- Success: {summary['success_count']}/{summary['trial_count']}",
            f"- Success rate: {summary['success_rate']}",
            f"- Mean elapsed: {summary['mean_elapsed_sec']} s",
            f"- Mean path length: {summary['mean_path_length_m']} m",
            f"- Emergency stops: {summary['total_emergency_stop_count']}",
            f"- Scan-threshold collisions: {summary['collision_count']}",
            '- Minimum obstacle distance: '
            f"{summary['minimum_obstacle_distance_m']} m",
            f"- Mean cmd period: {summary['mean_cmd_period_sec']} s",
            f"- Max cmd period: {summary['max_cmd_period_sec']} s",
            '',
            '## Mission Breakdown',
            '',
            *table,
            '',
            '## Interpretation',
            '',
            '- This result validates the current clean process-per-trial '
            'Nav2 mainline.',
            '- Collision is estimated from `/scan` range threshold, not from '
            'Gazebo physical contacts.',
            '- Dynamic obstacles are ROS-side scan-layer inputs in the '
            'current WSL/Fortress baseline.',
            '- Route graph computation is validated separately from full '
            'route-constrained execution.',
            '',
        ]),
        encoding='utf-8',
    )


def _svg_bar_chart(
    rows: list[dict[str, Any]],
    key: str,
    title: str,
    y_label: str,
) -> str:
    mission_rows = _mission_summary(rows)
    width = 880
    height = 360
    margin_left = 72
    margin_bottom = 82
    margin_top = 42
    plot_width = width - margin_left - 30
    plot_height = height - margin_top - margin_bottom
    values = [
        float(row[key])
        for row in mission_rows
        if isinstance(row.get(key), int | float)
    ]
    max_value = max(values) if values else 1.0
    max_value = max(max_value, 1.0)
    bar_gap = 24
    bar_width = max(
        42,
        (plot_width - bar_gap * (len(mission_rows) + 1))
        / max(1, len(mission_rows)),
    )
    elements = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="880" '
        'height="360" viewBox="0 0 880 360">',
        '<rect width="880" height="360" fill="#ffffff"/>',
        f'<text x="440" y="26" text-anchor="middle" '
        f'font-family="Arial" font-size="20">{title}</text>',
        f'<text x="20" y="165" transform="rotate(-90 20 165)" '
        f'text-anchor="middle" font-family="Arial" font-size="13">'
        f'{y_label}</text>',
        f'<line x1="{margin_left}" y1="{height - margin_bottom}" '
        f'x2="{width - 30}" y2="{height - margin_bottom}" '
        'stroke="#333"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" '
        f'x2="{margin_left}" y2="{height - margin_bottom}" '
        'stroke="#333"/>',
    ]
    for tick in range(5):
        value = max_value * tick / 4.0
        y = height - margin_bottom - (value / max_value) * plot_height
        elements.append(
            f'<line x1="{margin_left - 4}" y1="{y:.1f}" '
            f'x2="{width - 30}" y2="{y:.1f}" stroke="#e0e0e0"/>'
        )
        elements.append(
            f'<text x="{margin_left - 8}" y="{y + 4:.1f}" '
            f'text-anchor="end" font-family="Arial" font-size="11">'
            f'{value:.2f}</text>'
        )
    for index, row in enumerate(mission_rows):
        value = row.get(key)
        numeric = float(value) if isinstance(value, int | float) else 0.0
        x = margin_left + bar_gap + index * (bar_width + bar_gap)
        bar_height = (numeric / max_value) * plot_height
        y = height - margin_bottom - bar_height
        elements.append(
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_width:.1f}" '
            f'height="{bar_height:.1f}" fill="#2f6f8f"/>'
        )
        elements.append(
            f'<text x="{x + bar_width / 2:.1f}" y="{y - 6:.1f}" '
            f'text-anchor="middle" font-family="Arial" font-size="11">'
            f'{numeric:.2f}</text>'
        )
        label = str(row['mission_id']).replace('lab_', '')
        elements.append(
            f'<text x="{x + bar_width / 2:.1f}" '
            f'y="{height - margin_bottom + 18}" text-anchor="middle" '
            f'font-family="Arial" font-size="10">{label}</text>'
        )
    elements.append('</svg>')
    return '\n'.join(elements) + '\n'


def write_figures(rows: list[dict[str, Any]], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'mean_elapsed_sec.svg').write_text(
        _svg_bar_chart(
            rows,
            'mean_elapsed_sec',
            'Mean Mission Elapsed Time',
            'seconds',
        ),
        encoding='utf-8',
    )
    (directory / 'mean_path_length_m.svg').write_text(
        _svg_bar_chart(
            rows,
            'mean_path_length_m',
            'Mean Path Length',
            'meters',
        ),
        encoding='utf-8',
    )
    (directory / 'README.md').write_text(
        '\n'.join([
            '# Figures',
            '',
            '- `mean_elapsed_sec.svg`: per-mission mean task time.',
            '- `mean_path_length_m.svg`: per-mission mean path length.',
            '',
        ]),
        encoding='utf-8',
    )


def export_report_artifacts(
    input_path: Path,
    csv_path: Path,
    markdown_path: Path,
    figures_dir: Path,
) -> dict[str, Any]:
    rows = load_rows(input_path)
    summary = summarize(input_path)
    write_csv(rows, csv_path)
    write_markdown(summary, rows, markdown_path, input_path)
    write_figures(rows, figures_dir)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Summarize AIROS navigation trial JSONL.',
    )
    parser.add_argument('--input', default='log/airos_nav_trials.jsonl')
    parser.add_argument('--output', default='')
    parser.add_argument('--csv-output', default='')
    parser.add_argument('--markdown-output', default='')
    parser.add_argument('--figures-dir', default='')
    args = parser.parse_args()

    input_path = Path(args.input)
    if args.csv_output or args.markdown_output or args.figures_dir:
        summary = export_report_artifacts(
            input_path,
            Path(args.csv_output or 'results/single_floor_lab_summary.csv'),
            Path(
                args.markdown_output
                or 'results/single_floor_lab_summary.md'
            ),
            Path(args.figures_dir or 'results/figures'),
        )
    else:
        summary = summarize(input_path)
    text = json.dumps(summary, indent=2, ensure_ascii=False)
    print(text)
    if args.output:
        Path(args.output).write_text(text + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
