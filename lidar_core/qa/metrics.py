from __future__ import annotations

import math


def mae(truth: list[float], prediction: list[float]) -> float:
    return sum(abs(left - right) for left, right in zip(truth, prediction)) / max(len(truth), 1)


def rmse(truth: list[float], prediction: list[float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(truth, prediction)) / max(len(truth), 1))


def r2_score(truth: list[float], prediction: list[float]) -> float:
    if not truth:
        return 0.0
    mean_truth = sum(truth) / len(truth)
    total = sum((value - mean_truth) ** 2 for value in truth)
    residual = sum((left - right) ** 2 for left, right in zip(truth, prediction))
    if total == 0.0:
        return 0.0
    return 1.0 - residual / total


def classification_metrics(truth: list[int], prediction: list[int]) -> dict:
    true_positive = sum(1 for left, right in zip(truth, prediction) if left == 1 and right == 1)
    false_positive = sum(1 for left, right in zip(truth, prediction) if left == 0 and right == 1)
    false_negative = sum(1 for left, right in zip(truth, prediction) if left == 1 and right == 0)
    precision = true_positive / max(true_positive + false_positive, 1)
    recall = true_positive / max(true_positive + false_negative, 1)
    if precision + recall == 0.0:
        f1_score = 0.0
    else:
        f1_score = 2.0 * precision * recall / (precision + recall)
    return {
        "precision": precision,
        "recall": recall,
        "f1": f1_score,
        "true_positive": true_positive,
        "false_positive": false_positive,
        "false_negative": false_negative,
    }
