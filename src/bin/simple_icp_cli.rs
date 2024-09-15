use clap::Parser;
use nalgebra as na;
use rayon::prelude::*;
use simple_icp::point3d;
use simple_icp::point3d::Point3d;
use std::collections::HashMap;
use std::mem::ManuallyDrop;
use unbag_rs::ros1::msg::{Msg, PointCloud2};
use unbag_rs::ros1::Ros1Bag;

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

pub fn bytes_slice_to_data<ReturnType, const LENGTH: usize>(
    data: &[u8],
    start_idx: usize,
) -> (ReturnType, usize) {
    let mut raw: [u8; LENGTH] = [0; LENGTH];
    raw.copy_from_slice(&data[start_idx..start_idx + LENGTH]);
    (
        unsafe {
            std::mem::transmute_copy::<ManuallyDrop<[u8; LENGTH]>, ReturnType>(&ManuallyDrop::new(
                raw,
            ))
        },
        start_idx + LENGTH,
    )
}

fn pointcloud_to_vec_point3d(pcd: &PointCloud2) -> Vec<Point3d> {
    let name_to_offset: HashMap<String, usize> = pcd
        .fields
        .iter()
        .map(|f| (f.name.to_owned(), f.offset as usize))
        .collect();

    (0..pcd.width)
        .into_par_iter()
        .map(|i| {
            let idx = (i * pcd.point_step) as usize;
            let (x, _) = bytes_slice_to_data::<f32, 4>(&pcd.data, idx + name_to_offset["x"]);
            let (y, _) = bytes_slice_to_data::<f32, 4>(&pcd.data, idx + name_to_offset["y"]);
            let (z, _) = bytes_slice_to_data::<f32, 4>(&pcd.data, idx + name_to_offset["z"]);
            let (intensity, _) =
                bytes_slice_to_data::<f32, 4>(&pcd.data, idx + name_to_offset["intensity"]);
            // println!("{}", intensity);
            Point3d { intensity, x, y, z }
        })
        .collect()
}

#[derive(Parser)]
#[command(version, about, long_about = None)]
struct SimpleIcpCli {
    /// path to
    path: String,

    #[arg(long, default_value_t = 0)]
    start: usize,

    #[arg(long)]
    topic: Option<String>,
}

fn main() {
    let cli = SimpleIcpCli::parse();
    let bag = Ros1Bag::new(&cli.path);
    if bag
        .topic_to_type
        .values()
        .filter(|k| k.contains("sensor_msgs/PointCloud2"))
        .count()
        > 1
        && cli.topic.is_none()
    {
        println!(
            "More than one topic in the bag. Please use --topic to specify which lidar topic."
        );
        for kv in bag.topic_to_type {
            println!("{}: {}", kv.0, kv.1);
        }
        return;
    }
    let topics = if let Some(topic) = cli.topic {
        vec![topic]
    } else {
        vec![]
    };

    let recording = rerun::RecordingStreamBuilder::new("visualize rosbag")
        .spawn()
        .expect("output path error");

    let min_val = 0.0;
    let max_val = 255.0;
    let mut pipeline = simple_icp::icp_pipeline::IcpPipeline::default_values();
    for (i, msg) in bag.read_messages(&topics).enumerate() {
        if i < cli.start {
            continue;
        }
        if let Msg::PointCloud2(pointcloud) = msg {
            let points = pointcloud_to_vec_point3d(&pointcloud);
            pipeline.process_frame(&points);
            let points = pipeline.get_last_batch_points();
            let timestamp = pointcloud.header.sec as f64 + (pointcloud.header.nsec as f64 / 1e9);
            if timestamp != 0.0 {
                recording.set_time_seconds("stable_time", timestamp);
            }
            let colors = get_colors_for_points(points, min_val, max_val, 255);
            recording
                .log(
                    "",
                    &rerun::Points3D::new(points.iter().map(|p| (p.x, p.y, p.z)))
                        .with_radii([0.05])
                        .with_colors(colors),
                )
                .unwrap();
            recording
                .log(
                    "lidar_pose",
                    &na_to_rerun(&pipeline.t_origin_current).with_axis_length(1.0),
                )
                .unwrap();
        }
    }
}
