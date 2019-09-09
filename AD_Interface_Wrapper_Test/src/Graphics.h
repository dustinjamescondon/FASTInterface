#pragma once
#include <SFML/Graphics.hpp>
#include "..\..\AD_Interface_Wrapper(ver2)\eigen\Eigen\Dense"


// Projects the node positions onto the yz plane and displays them on the screen as dots
void RenderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Eigen::Vector3d& hubPos, double pixelsPerMeter, int nNodes);
