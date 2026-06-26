# this is the first step i'm doing as it's the single file where every value i need to tune stays, esp with the assignment requiremnts, if the DBSCAN eps needs changing, then i can change it here and nowhere else, and this is just a mental note to myself

from dataclasses import dataclass

@dataclass
class Config:
    """Central configuration class for the point cloud processing pipeline."""
    
    # Preprocessing parameters
    voxel_size: float = 0.02
    # voxel_size: this is where each 3d cube of this side-length (in m) collapses to a single point on the centroid.
    # eagle is a very large outdoor scan, based on what i saw online, 0.02m = 2cm cubes could work well, maybe i can make it 0.05 = 5 cm cubes, and this would mostly....
    # too many points-> increase, shape gets blocky-> decrease
    
    sor_nb_neighbors: int = 30
    # sor_nb_neighbors: this is the number of neighbors to analyze for each point when removing outliers. A higher value means more points are considered, which can lead to better noise removal but may also remove valid points. A lower value may retain more points but could leave more noise in the data.
    # its means distance, more nieghbours = smoother and cleaner est. , slow computation
    
    sor_std_ratio: float = 2.0
    # SOR removes points whose mean neighbour distance exceeds
    # global_mean + std_ratio * std_deviation.
    
    # normal estimation parameters
    normal_radius: float = 0.1
    
    normal_max_nn: int = 30
    # hard limit on neighbours even if more fall within normal_radius.
    #prevents very dense regions from dominating the computtation time and effort....~~
    
    # clustering (DBSCAN) ~~ smth new to learn 
    
    clustering_eps: float = 0.05
    # clustering_eps: this is the maximum distance between two points for them to be considered as
    # too small -> everything becomes nouse
    # tooo large -> all clusters merge into one, and the noise points are also merged into the clusters, which is not good.
    
    clustering_min_points: int = 50
    # clustering_min_points: minimum local density DBSCAN needs before treating a region as a real cluster.
    
    # gotta tune this first if clustering is not working well, then i can tune the voxel_size and sor_nb_neighbors and sor_std_ratio, but first i need to tune this if it looks wrong.
    
    # outputs
    output_dir: str = "docs/renders"
    
    render_subsample: int = 8000
    
    #Matplotlis slowsn down badly past ~10k scattered points in 3d.
    # we randomly subsample to this count for rendering only..... 
    # the actual processing is done on the full point cloud, but for visualization, we limit the number of points to render to avoid performance issues.
    
    
