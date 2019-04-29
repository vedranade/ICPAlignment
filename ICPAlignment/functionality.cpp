#include <GL/glew.h>
#include <GL/freeglut.h>

#include <glm/glm.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <istream>
#include <iostream>
#include "functionality.h"

const int dim = 3;
const int max_leaf = 10;

Aligner::Aligner(Eigen::MatrixXd d, Eigen::MatrixXd m) : firstModel_verts(d), secondModel_verts(m) 
{
	N_data = d.rows();
}

bool Aligner::perform_icp() 
{

	model_kd_tree = new kd_tree_t(dim, secondModel_verts, max_leaf);

	while (check_iter()) 
	{
		//std::cout << "Iteration: " << iter_counter << ", Error: " << error << std::endl;
	}
	if (iteration_has_converged) 
	{
		//std::cout << "Iteration converged!" << std::endl;
		return true;
	}
	else 
	{
		//std::cout << "Iteration did not converge.." << std::endl;
		return false;
	}
}

bool Aligner::check_iter() 
{
	double error_diff = std::abs(error - old_error);

	if ((iter_counter < max_it) && !(error_diff < threshold)) 
	{
		//Calculate closest points using KD tree search:
		pointSearch();

		// Compute the optimal transformation of the data
		translation = Eigen::Vector3d::Zero();
		rotation = Eigen::Matrix3d::Zero();

		compute_registration(translation, rotation);

		// Transform the first model:
		firstModel_verts = firstModel_verts * rotation.transpose();
		firstModel_verts = firstModel_verts + translation.transpose().replicate(N_data, 1);

		//// Store accumulative transformations
		//final_rotation = rotation * final_rotation;
		//final_translation += translation;

		//// Save the error
		//old_error = error;
		//error = calculateError(translation, rotation);

		iter_counter++;

	}
	else if (error_diff < threshold) 
	{
		iteration_has_converged = true;
		return false;
	}
	else if (iter_counter == max_it)
		return false;

	return true;
}

void Aligner::pointSearch() 
{
	point_correspondence.clear();
	weights.clear();

	const size_t num_results = 1;
	std::vector<size_t> nn_index(num_results);
	std::vector<double> nn_distance(num_results);
	std::vector<double> distances(N_data);
	nanoflann::KNNResultSet<double> result_set(num_results);
	double mean = 0;

	for (int i = 0; i < N_data; i++)
	{
		//For each point in first model, look for the closest point in the second model:
		std::vector<double> query_pt = { firstModel_verts(i, 0),
										firstModel_verts(i, 1),
										firstModel_verts(i, 2) };

		//Sets the point correspondence and point distance:
		result_set.init(&nn_index[0], &nn_distance[0]);

		model_kd_tree->index->findNeighbors(result_set, &query_pt[0], nanoflann::SearchParams(10));
		point_correspondence[i] = nn_index[0];
		distances[i] = nn_distance[0];
		mean += nn_distance[0];
	} 
	mean /= N_data;

	// Calculate variance and std. dev. of distances:
	double variance = 0;
	
	for (int i = 0; i < N_data; i++) 
		variance += ((distances[i] - mean)*(distances[i] - mean));
	variance /= N_data;

	double std_deviation = sqrt(variance);
	double rejected = 0;

	// Reject point-pairs based on threshold distance rule
	double comp;
	
	for (int i = 0; i < N_data; i++) 
	{
		comp = 1.5 * std_deviation;
		if (std::abs(distances[i] - mean) > comp) 
		{
			point_correspondence.erase(i);
			rejected++;
		}
	}

	//std::cout << "Rejected " << rejected / N_data << "% of the sample point-pairs." << std::endl;

	// Find max distance between points
	std::vector<double>::iterator max_dist_it;
	max_dist_it = std::max_element(distances.begin(), distances.end());
}

void Aligner::compute_registration(Eigen::Vector3d &translation, Eigen::Matrix3d &rotation)
{
	size_t N_data = firstModel_verts.rows();
	size_t N_model = secondModel_verts.rows();
	size_t N_pc = point_correspondence.size();

	// Divide sum of each coordinate by number of data points to obtain center of mass:
	Eigen::Vector3d data_COM = firstModel_verts.colwise().sum() / N_data;
	Eigen::Vector3d model_COM = secondModel_verts.colwise().sum() / N_model;

	// Construct covariance matrix
	Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
	for (std::map<int, int>::iterator it = point_correspondence.begin(); it != point_correspondence.end(); ++it)
	{
		int index = it->first;
		covariance_matrix += (firstModel_verts.row(index).transpose() * secondModel_verts.row(it->second));
	}
	covariance_matrix /= N_pc;
	covariance_matrix -= (data_COM * model_COM.transpose());

	// Construct Q-matrix
	Eigen::Matrix3d A = covariance_matrix - covariance_matrix.transpose();
	Eigen::Vector3d delta;
	delta << A(1, 2), A(2, 0), A(0, 1);

	Eigen::Matrix4d Q;
	double Q_trace = covariance_matrix.trace();
	Q(0, 0) = Q_trace;
	Q.block(1, 0, 3, 1) = delta;
	Q.block(0, 1, 1, 3) = delta.transpose();
	Q.block(1, 1, 3, 3) = covariance_matrix + covariance_matrix.transpose() - Q_trace * Eigen::MatrixXd::Identity(3, 3);

	// Find optimal unit quaternion
	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(Q);
	Eigen::MatrixXd::Index max_ev_index;
	eigen_solver.eigenvalues().real().cwiseAbs().maxCoeff(&max_ev_index);
	Eigen::Vector4d q_optimal = eigen_solver.eigenvectors().real().col(max_ev_index);

	// Store the calculate transformation:
	calculateQMatrix(q_optimal, rotation);
	translation = model_COM - rotation * data_COM;
}

void Aligner::calculateQMatrix(Eigen::Vector4d q, Eigen::Matrix3d &R) 
{
	//Hard coding values for now:
	R(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	R(1, 0) = 2 * (q[1] * q[2] + q[0] * q[3]);
	R(2, 0) = 2 * (q[1] * q[3] - q[0] * q[2]);

	R(0, 1) = 2 * (q[1] * q[2] - q[0] * q[3]);
	R(1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	R(2, 1) = 2 * (q[2] * q[3] + q[0] * q[1]);

	R(0, 2) = 2 * (q[1] * q[3] + q[0] * q[2]);
	R(1, 2) = 2 * (q[2] * q[3] - q[0] * q[1]);
	R(2, 2) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

std::vector<Vertex> loadOBJ(std::istream& in)
{
	std::vector< Vertex > verts;

	std::vector< glm::vec4 > positions(1, glm::vec4(0, 0, 0, 0));
	std::vector< glm::vec3 > texture_coords(1, glm::vec3(0, 0, 0));
	std::vector< glm::vec3 > normals(1, glm::vec3(0, 0, 0));

	std::string lineStr;

	while (std::getline(in, lineStr))
	{
		std::istringstream lineSS(lineStr);
		std::string lineType;
		lineSS >> lineType;

		// vertex
		if (lineType == "v")
		{
			float x = 0, y = 0, z = 0, w = 1;
			lineSS >> x >> y >> z >> w;
			positions.push_back(glm::vec4(x, y, z, w));
		}

		// texture
		if (lineType == "vt")
		{
			float u = 0, v = 0, w = 0;
			lineSS >> u >> v >> w;
			texture_coords.push_back(glm::vec3(u, v, w));
		}

		// normal
		if (lineType == "vn")
		{
			float i = 0, j = 0, k = 0;
			lineSS >> i >> j >> k;
			normals.push_back(glm::normalize(glm::vec3(i, j, k)));
		}

		// polygon
		if (lineType == "f")
		{
			std::vector< VertRef > refs;
			std::string refStr;
			while (lineSS >> refStr)
			{
				std::istringstream ref(refStr);
				std::string vStr, vtStr, vnStr;
				std::getline(ref, vStr, '/');
				std::getline(ref, vtStr, '/');
				std::getline(ref, vnStr, '/');
				int v = atoi(vStr.c_str());
				int vt = atoi(vtStr.c_str());
				int vn = atoi(vnStr.c_str());
				v = (v >= 0 ? v : positions.size() + v);
				vt = (vt >= 0 ? vt : texture_coords.size() + vt);
				vn = (vn >= 0 ? vn : normals.size() + vn);
				refs.push_back(VertRef(v, vt, vn));
			}

			// triangulate, assuming n>3-gons are convex and coplanar
			for (size_t i = 1; i + 1 < refs.size(); ++i)
			{
				const VertRef* p[3] = { &refs[0], &refs[i], &refs[i + 1] };

				glm::vec3 U(positions[p[1]->v] - positions[p[0]->v]);
				glm::vec3 V(positions[p[2]->v] - positions[p[0]->v]);
				glm::vec3 faceNormal = glm::normalize(glm::cross(U, V));

				for (size_t j = 0; j < 3; ++j)
				{
					Vertex vert;
					vert.position = glm::vec3(positions[p[j]->v]);
					vert.texture_coord = glm::vec2(texture_coords[p[j]->vt]);
					vert.normal = (p[j]->vn != 0 ? normals[p[j]->vn] : faceNormal);
					verts.push_back(vert);
				}
			}
		}
	}

	return verts;
}
