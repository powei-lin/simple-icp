use crate::{
    point3d::{self, Point3d},
    voxel_hash_map::{self, Voxel},
};
use nalgebra as na;
use rayon::prelude::*;
use std::collections::HashMap;

pub fn voxel_downsample(point_cloud: &[point3d::Point3d], voxel_size: f32) -> Vec<Point3d> {
    let grid: HashMap<Voxel, point3d::Point3d> = point_cloud
        .par_iter()
        .map(|pt| (point_to_voxel(pt, voxel_size), *pt))
        .collect();
    return grid.values().cloned().collect();
}

pub fn point_to_voxel(point: &point3d::Point3d, voxel_size: f32) -> voxel_hash_map::Voxel {
    Voxel::new(
        (point.x / voxel_size).floor() as i32,
        (point.y / voxel_size).floor() as i32,
        (point.z / voxel_size).floor() as i32,
    )
}

pub fn na_vec_to_voxel(point: &na::Vector3<f64>, voxel_size: f64) -> voxel_hash_map::Voxel {
    Voxel::new(
        (point.x / voxel_size).floor() as i32,
        (point.y / voxel_size).floor() as i32,
        (point.z / voxel_size).floor() as i32,
    )
}
