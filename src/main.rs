use glob::glob;
use nalgebra as na;
use serde_json;
use simple_icp::{icp_pipeline::IcpPipeline, point3d, voxel_util::voxel_downsample};
use std::collections::HashMap;
use std::{
    fs,
    io::{BufReader, Write},
    ptr::addr_eq,
};

pub fn get_colors_for_points(
    points: &[point3d::Point3d],
    min_val: f32,
    max_val: f32,
    alpha: u8,
) -> Vec<(u8, u8, u8, u8)> {
    let g = colorous::TURBO;
    points
        .iter()
        .map(|j| {
            let c = g.eval_continuous(((j.intensity - min_val) / (max_val - min_val)).into());
            (c.r, c.g, c.b, alpha)
        })
        .collect()
}
pub fn get_colors_for_na_points(
    points: &[na::Vector3<f64>],
    min_val: f32,
    max_val: f32,
    alpha: u8,
) -> Vec<(u8, u8, u8, u8)> {
    let g = colorous::TURBO;
    points
        .iter()
        .map(|j| {
            let c = g.eval_continuous(((j.z as f32 - min_val) / (max_val - min_val)).into());
            (c.r, c.g, c.b, alpha)
        })
        .collect()
}

fn na_to_rerun(transform: &na::Isometry3<f64>) -> rerun::Transform3D {
    rerun::Transform3D::from_translation_rotation(
        [
            transform.translation.x as f32,
            transform.translation.y as f32,
            transform.translation.z as f32,
        ],
        rerun::Quaternion::from_wxyz([
            transform.rotation.w as f32,
            transform.rotation.i as f32,
            transform.rotation.j as f32,
            transform.rotation.k as f32,
        ]),
    )
}

fn main() {
    let min_val = -80.0;
    let max_val = -20.0;
    // let min_val = -10.0;
    // let max_val = 10.0;

    let pcd_paths = "./pcd/*.pcd";
    // let recording = rerun::RecordingStreamBuilder::new("visualize rosbag")
    //     .save("output.rrd")
    //     .expect("output path error");
    let recording = rerun::RecordingStreamBuilder::new("visualize rosbag")
        .spawn()
        .expect("output path error");

    let config = simple_icp::config::Config::default();
    let mut pipeline = simple_icp::icp_pipeline::IcpPipeline::default();

    let mut prev_points = Vec::<point3d::Point3d>::new();

    for (i, entry) in glob(pcd_paths)
        .expect("Failed to read glob pattern")
        .enumerate()
    {
        if i < 700 {
            continue;
        }
        println!("{}", i);
        if let Ok(path) = entry {
            let points = point3d::read_pcd_to_points(path.to_str().unwrap());
            pipeline.process_frame(&points);
            // let points = point3d::clip_point_cloud_by_distance(&points, 0.1, 200.0);
            let points = pipeline.get_last_batch_points();

            let timestamp_ns = path
                .file_stem()
                .unwrap()
                .to_str()
                .unwrap()
                .parse::<u64>()
                .unwrap();
            recording.set_time_seconds("stable_time", timestamp_ns as f64 / 1e9);
            // let colors = get_colors_for_na_points(&points, min_val, max_val, 255);
            let colors = get_colors_for_points(points, min_val, max_val, 255);
            recording
                .log(
                    "",
                    &rerun::Points3D::new(
                        points.iter().map(|p| (p.x as f32, p.y as f32, p.z as f32)),
                    )
                    .with_radii([0.1])
                    .with_colors(colors),
                )
                .unwrap();
            recording
                .log(
                    "lidar_pose",
                    &na_to_rerun(&pipeline.t_origin_current).with_axis_length(10.0),
                )
                .unwrap();
        }
    }
}
