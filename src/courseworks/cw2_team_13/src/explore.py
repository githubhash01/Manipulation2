import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt


# -------------------------
# REMOVE TABLE (3D)
# -------------------------
def remove_table(cloud):
    pts = np.asarray(cloud.points)
    cols = np.asarray(cloud.colors)

    green_mask = (
        (cols[:, 1] > 0.4)
        & (cols[:, 1] > cols[:, 0])
        & (cols[:, 1] > cols[:, 2])
    )

    table_z = np.median(pts[green_mask][:, 2])
    print(f"Estimated table height: {table_z:.3f}")

    height_mask = pts[:, 2] > (table_z + 0.01)
    non_green_mask = ~green_mask

    final_mask = height_mask & non_green_mask

    filtered = o3d.geometry.PointCloud()
    filtered.points = o3d.utility.Vector3dVector(pts[final_mask])
    filtered.colors = o3d.utility.Vector3dVector(cols[final_mask])

    print(f"Filtered cloud: {len(filtered.points)} points")

    return filtered


# -------------------------
# TOP-DOWN PROJECTION + HEIGHT
# -------------------------
def pcd_to_topdown_image_with_height(cloud, grid_size=256):
    pts = np.asarray(cloud.points)
    cols = np.asarray(cloud.colors)

    min_xy = pts[:, :2].min(axis=0)
    max_xy = pts[:, :2].max(axis=0)

    scale = max_xy - min_xy
    scale[scale == 0] = 1e-6

    norm_xy = (pts[:, :2] - min_xy) / scale

    px = (norm_xy[:, 0] * (grid_size - 1)).astype(int)
    py = (norm_xy[:, 1] * (grid_size - 1)).astype(int)

    img = np.zeros((grid_size, grid_size, 3))
    height_map = -np.inf * np.ones((grid_size, grid_size))

    for i in range(len(pts)):
        x = px[i]
        y = py[i]
        z = pts[i, 2]

        if z > height_map[y, x]:
            height_map[y, x] = z
            img[y, x] = cols[i]

    return img, height_map


# -------------------------
# REMOVE ROBOT
# -------------------------
def filter_robot(img, height_map, radius=40):
    h, w, _ = img.shape
    cx, cy = w // 2, h // 2

    img[cy - radius:cy + radius, cx - radius:cx + radius] = [1, 1, 1]
    height_map[cy - radius:cy + radius, cx - radius:cx + radius] = -np.inf

    return img, height_map


# -------------------------
# WHITE BACKGROUND
# -------------------------
def make_white_background(img):
    mask = (img.sum(axis=2) == 0)
    img[mask] = [1.0, 1.0, 1.0]
    return img


# -------------------------
# BINARY MASK
# -------------------------
def image_to_binary(img):
    mask = np.linalg.norm(img - 1.0, axis=2) > 0.1
    return mask.astype(np.uint8)


# -------------------------
# CLUSTERING
# -------------------------
def cluster_image(img, height_map):
    binary = image_to_binary(img)
    num_labels, labels = cv2.connectedComponents(binary)

    clusters = []

    for label in range(1, num_labels):
        ys, xs = np.where(labels == label)

        if len(xs) < 50:
            continue

        heights = height_map[ys, xs]
        heights = heights[heights > -np.inf]

        avg_height = np.mean(heights) if len(heights) > 0 else 0
        avg_color = np.mean(img[ys, xs], axis=0)

        clusters.append(
            {
                'label': label,
                'size': len(xs),
                'centroid': (np.mean(xs), np.mean(ys)),
                'height': avg_height,
                'color': avg_color,
            }
        )

    return clusters, labels


# -------------------------
# BASKET DETECTION (FROM ORIGINAL)
# -------------------------
def detect_basket_region(combined, grid_size=256):
    img_full, _ = pcd_to_topdown_image_with_height(combined, grid_size)

    r = img_full[:, :, 0]
    g = img_full[:, :, 1]
    b = img_full[:, :, 2]

    brown_mask = (r > 0.4) & (g < 0.35) & (b < 0.35)
    brown_mask = brown_mask.astype(np.uint8)

    num_labels, labels = cv2.connectedComponents(brown_mask)

    max_size = 0
    basket_label = -1

    for l in range(1, num_labels):
        size = np.sum(labels == l)
        if size > max_size:
            max_size = size
            basket_label = l

    basket_mask = labels == basket_label
    return basket_mask


# -------------------------
# CLASSIFICATION
# -------------------------
def classify_clusters(clusters, labels, img, basket_mask):
    classified = []

    for c in clusters:
        label_id = c['label']
        color = c['color']
        cluster_mask = labels == label_id

        # Basket (override)
        overlap = np.sum(cluster_mask & basket_mask)

        if overlap > 50:
            ctype = 'basket'
        elif np.mean(color) < 0.2:
            ctype = 'obstacle'
        else:
            ys, xs = np.where(cluster_mask)
            y0, y1 = ys.min(), ys.max()
            x0, x1 = xs.min(), xs.max()

            crop = cluster_mask[y0:y1 + 1, x0:x1 + 1]

            h, w = crop.shape
            center = crop[h // 3:2 * h // 3, w // 3:2 * w // 3]

            if center.mean() < 0.2:
                ctype = 'nought'
            else:
                ctype = 'cross'

        c['type'] = ctype
        classified.append(c)

    return classified


# ----
# OUTPUT
# ----
def plan_mission(clusters):
    """
    - Counts noughts and crosses
    - Picks the more common type (ties go to cross)
    - Finds the one closest to the basket
    - Returns (object_pose, goal_pose) as (x, y) centroids, or (None, None)
    """
    noughts = [c for c in clusters if c['type'] == 'nought']
    crosses = [c for c in clusters if c['type'] == 'cross']
    baskets = [c for c in clusters if c['type'] == 'basket']

    if not baskets or (not noughts and not crosses):
        return None, None

    basket = max(baskets, key=lambda b: b['size'])
    bx, by = basket['centroid']

    candidates = crosses if len(crosses) >= len(noughts) else noughts

    def dist_sq(c):
        cx, cy = c['centroid']
        return (cx - bx) ** 2 + (cy - by) ** 2

    chosen = min(candidates, key=dist_sq)

    object_pose = chosen['centroid']
    goal_pose = basket['centroid']

    print(f"plan_mission: {len(noughts)} noughts, {len(crosses)} crosses")
    print(f"  picking {chosen['type']} at {object_pose}")
    print(f"  basket at {goal_pose}")

    return object_pose, goal_pose


# -------------------------
# VISUALIZATION
# -------------------------
def visualize_final(img, clusters, object_pose=None, goal_pose=None):
    plt.figure(figsize=(6, 6))
    plt.imshow(img)
    plt.title('Final Classification')
    plt.axis('off')

    for c in clusters:
        cx, cy = c['centroid']

        plt.scatter(cx, cy, c='black', s=60, marker='x')

        label = f"{c['type']} ({c['height']:.2f})"

        plt.text(
            cx + 5,
            cy + 5,
            label,
            color='black',
            fontsize=9,
            bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
        )

    # Highlight the chosen pick (green) and place target (magenta)
    if object_pose is not None and goal_pose is not None:
        ox, oy = object_pose
        gx, gy = goal_pose
        plt.plot([ox, gx], [oy, gy], 'k--', linewidth=1.5, alpha=0.7)
        plt.scatter([ox], [oy], s=260, facecolors='none',
                    edgecolors='lime', linewidths=2.5, label='pick')
        plt.scatter([gx], [gy], s=260, facecolors='none',
                    edgecolors='magenta', linewidths=2.5, label='place')
        plt.legend(loc='lower right')

    plt.show()


def pipeline(point_cloud):

    pts_c = np.asarray(point_cloud.points)

    print("\n>>> VIS 1: Combined cloud")
    o3d.visualization.draw_geometries([point_cloud])

    # Step 1: Remove table
    filtered = remove_table(point_cloud)

    print("\n>>> VIS 2: Table removed")
    o3d.visualization.draw_geometries([filtered])

    # Step 1.5: Detect basket region (for later classification)
    basket_mask = detect_basket_region(point_cloud)

    # Step 2: Top-down projection + height
    img, height_map = pcd_to_topdown_image_with_height(filtered)

    # Step 3: White background + filter robot
    img = make_white_background(img)
    img, height_map = filter_robot(img, height_map)

    # Step 4: Clustering
    clusters, labels = cluster_image(img, height_map)

    # Step 5: Classification
    clusters = classify_clusters(clusters, labels, img, basket_mask)

    # Step 6: Mission plan
    object_pose, goal_pose = plan_mission(clusters)

    visualize_final(img, clusters, object_pose, goal_pose)
    print("\n>>> CLUSTERS FOUND:")
    for i, c in enumerate(clusters):
        print(f"{i}: {c['type']} | height={c['height']:.3f} | size={c['size']}")


def main():
    combined = o3d.io.read_point_cloud('/tmp/cw2_scan_combined.pcd')
    pipeline(combined)


if __name__ == "__main__":
    main()