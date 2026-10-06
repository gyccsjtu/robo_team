"""Fresh green-person proof can support its currently authorized tracker."""
import math


def allowed(target_id, color, verified, miss, observed_s, now):
    return (target_id == 't0' and color == 'green' and verified is True
            and miss == 0 and math.isfinite(observed_s) and math.isfinite(now)
            and observed_s > 0 and 0 <= now-observed_s <= 1.)
