#[cfg(test)]
mod tests {
    use super::super::pgo::PoseGraphOptimizer;
    use tiny_solver::na;

    #[test]
    fn test_pgo_simple_graph() {
        let mut pgo = PoseGraphOptimizer::new();

        // Keyframe 0: Identity (Fixed)
        let pose0 = na::Isometry3::identity();
        pgo.add_keyframe(0, pose0, true);

        // Keyframe 1: 1m forward (Initial guess with error)
        // True pose: 1.0, 0, 0
        // Initial guess: 1.1, 0, 0
        let pose1_guess = na::Isometry3::translation(1.1, 0.0, 0.0);
        pgo.add_keyframe(1, pose1_guess, false);

        // Constraint 0->1: 1m forward
        let rel_pose_01 = na::Isometry3::translation(1.0, 0.0, 0.0);
        pgo.add_odometry_constraint(0, 1, rel_pose_01);

        // Optimize
        pgo.optimize();

        // Check result
        let optimized_pose1 = pgo.get_optimized_pose(1).unwrap();
        let diff = (optimized_pose1.translation.vector - na::Vector3::new(1.0, 0.0, 0.0)).norm();
        
        println!("Optimized Pose 1: {:?}", optimized_pose1);
        println!("Diff: {}", diff);

        assert!(diff < 1e-3, "Optimization should converge to true pose");
    }

    #[test]
    fn test_pgo_loop_closure() {
        let mut pgo = PoseGraphOptimizer::new();

        // Square path: 0 -> 1 -> 2 -> 3 -> 0
        // Side length: 10m

        // True poses
        let p0 = na::Isometry3::identity();
        let _p1 = na::Isometry3::translation(10.0, 0.0, 0.0);
        let _p2 = na::Isometry3::translation(10.0, 10.0, 0.0);
        let p3 = na::Isometry3::translation(0.0, 10.0, 0.0);

        // Initial guesses (with drift)
        pgo.add_keyframe(0, p0, true);
        pgo.add_keyframe(1, na::Isometry3::translation(10.1, 0.0, 0.0), false);
        pgo.add_keyframe(2, na::Isometry3::translation(10.2, 10.1, 0.0), false);
        pgo.add_keyframe(3, na::Isometry3::translation(0.1, 10.2, 0.0), false);

        // Odometry constraints
        pgo.add_odometry_constraint(0, 1, na::Isometry3::translation(10.0, 0.0, 0.0));
        pgo.add_odometry_constraint(1, 2, na::Isometry3::translation(0.0, 10.0, 0.0));
        pgo.add_odometry_constraint(2, 3, na::Isometry3::translation(-10.0, 0.0, 0.0));
        
        // Loop closure constraint: 3 -> 0
        pgo.add_odometry_constraint(3, 0, na::Isometry3::translation(0.0, -10.0, 0.0));

        // Optimize
        pgo.optimize();

        // Check loop closure error
        let opt_p3 = pgo.get_optimized_pose(3).unwrap();
        let diff = (opt_p3.translation.vector - p3.translation.vector).norm();
        println!("Optimized Pose 3: {:?}", opt_p3);
        println!("Diff: {}", diff);
        
        assert!(diff < 1e-2, "Loop closure should correct drift");
    }
}
