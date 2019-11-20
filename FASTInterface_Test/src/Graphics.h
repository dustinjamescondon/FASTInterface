#pragma once
#include <SFML/Graphics.hpp>
//#include <boost/numeric/ublas/vector.hpp>
#include <Eigen/Dense>

//using Vector3d = boost::numeric::ublas::vector<double, boost::numeric::ublas::bounded_array<double, 3> >;
// Projects the node positions onto the yz plane and displays them on the screen as dots
void RenderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Eigen::Vector3d& hubPos, double pixelsPerMeter, int nNodes);
