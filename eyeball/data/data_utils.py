import logging
from datetime import datetime
from glob import glob
from typing import Any, Dict, List, Optional



def get_ms_ts_for_abs_ts(ts: float) -> Optional[float]:
    """Convert absolute timestamp to milliseconds.

    Also check if if the abs timestamp is valid by looking at the year.
    Return None if the timestamp is not valid.
    """
    maybe_ts_ms = None
    if ts > 1e12:
        maybe_ts_ms = ts
    elif ts > 1e9:
        maybe_ts_ms = ts * 1e3
    elif ts > 1e6:
        maybe_ts_ms = ts * 1e6
    else:
        logging.warning(f"Unknown timestamp unit: {ts}")
        return None
    # convert to datetime and check if it's in the 2025 year
    dt = datetime.fromtimestamp(maybe_ts_ms / 1000)
    year_valid = 2020 <= dt.year <= 2030
    if not year_valid:
        logging.warning(f"Timestamp year is not valid: {dt.year}")
        return None
    return maybe_ts_ms


def recusive_flatten(dictionary: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten a nested dictionary, putting : to preserve information about the original heiarchy.

    ":" is used to preserve information about the original heiarchy.

    ex = {"a": {"b": 1, "c": {"d": 2}}}
    recusive_flatten(ex) -> {"a-b": 1, "a-c-d": 2}
    """
    result = {}
    for k, v in dictionary.items():
        assert "-" not in k
        if isinstance(v, dict):
            flat_v = recusive_flatten(v)
            for flat_k, _v in flat_v.items():
                result[f"{k}-{flat_k}"] = _v
        else:
            result[k] = v
    return result


def reverse_flatten(dictionary: Dict[str, Any]) -> Dict[str, Any]:
    """Reverse the flatten operation.

    ex = {"a-b": 1, "a-c-d": 2}
    reverse_flatten(ex) -> {"a": {"b": 1, "c": {"d": 2}}}

    ex = {"b": 1, "a-c": 2}
    reverse_flatten(ex) -> {"b": 1, "a": {"c": 2}}
    """
    result = {}
    for k, v in dictionary.items():
        keys = k.split("-")
        current = result
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        current[keys[-1]] = v
    return result


def flatten_dict(d, parent_key="", sep="_"):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def flatten_list_dict(data: List[Dict[str, Any]]) -> Dict[str, Any]:
    flat_keys = recusive_flatten(data[0]).keys()
    result = {k: [] for k in flat_keys}
    for d in data:
        flat_d = recusive_flatten(d)
        for k in flat_keys:
            result[k].append(flat_d[k])
    return result
