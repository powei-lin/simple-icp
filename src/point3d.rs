use nalgebra as na;
use pcd_rs::PcdDeserialize;
use rayon::prelude::*;

#[derive(PcdDeserialize, Debug, Clone, Copy)]
pub struct Point3d {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
}

impl Point3d {
    pub fn square(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn to_na_vec_f64(&self) -> na::Vector3<f64> {
        na::Vector3::<f64>::new(self.x as f64, self.y as f64, self.z as f64)
    }
    pub fn to_na_point3_f64(&self) -> na::Point3<f64> {
        na::Point3::<f64>::new(self.x as f64, self.y as f64, self.z as f64)
    }
}

pub fn read_pcd_to_points(pcd_path: &str) -> Vec<Point3d> {
    let reader = pcd_rs::Reader::open(pcd_path).unwrap();
    let points: Result<Vec<Point3d>, _> = reader.collect();
    points.expect("wrong format")
}

pub fn clip_point_cloud_by_distance(
    point_cloud: &[Point3d],
    min_distance: f32,
    max_distance: f32,
) -> Vec<Point3d> {
    let min2 = min_distance * min_distance;
    let max2 = max_distance * max_distance;
    point_cloud
        .par_iter()
        .filter_map(|pt| {
            let s = pt.square();
            if s < min2 || s > max2 {
                None
            } else {
                Some(pt.clone())
            }
        })
        .collect()
}
