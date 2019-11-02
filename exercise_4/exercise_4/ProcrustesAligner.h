#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean);

		// To apply the pose to point x on shape X in the case of Procrustes, we execute:
		// 1. Translation of a point to the shape Y: x' = x + t
		// 2. Rotation of the point around the mean of shape Y: 
		//    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)
		
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = rotation * translation - rotation * targetMean + targetMean;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
		for (Vector3f p : points) mean += p;
		mean = mean * (1 / (float)points.size());
		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		
		unsigned int n = sourcePoints.size();
		MatrixX3f source = MatrixXf::Zero(n, 3);
		MatrixX3f target = MatrixXf::Zero(n, 3);
		for (unsigned int i = 0; i < n; i++) {
			source(i, 0) = sourcePoints[i].x() - sourceMean.x();
			source(i, 1) = sourcePoints[i].y() - sourceMean.y();
			source(i, 2) = sourcePoints[i].z() - sourceMean.z();

			target(i, 0) = targetPoints[i].x() - targetMean.x();
			target(i, 1) = targetPoints[i].y() - targetMean.y();
			target(i, 2) = targetPoints[i].z() - targetMean.z();
		}

		Matrix3f m = target.transpose() * source;
		JacobiSVD<MatrixXf> svd(m, ComputeFullU | ComputeFullV);
		Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
		return R;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
		// TODO: Compute the translation vector from source to target points.
		
		return targetMean - sourceMean;
	}
};