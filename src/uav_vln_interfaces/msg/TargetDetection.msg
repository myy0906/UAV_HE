# ============================================================
# TargetDetection
#
# 2D visual detection result.
# Pixel coordinates are normalized to [0, 1] so that RGB and
# depth images may have different resolutions.
# ============================================================

uint64 timestamp_us

bool detected
string label
float32 confidence

float32 center_u_norm
float32 center_v_norm

float32 bbox_xmin_norm
float32 bbox_ymin_norm
float32 bbox_xmax_norm
float32 bbox_ymax_norm

float32 area_ratio
