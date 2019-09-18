#include "Graphics.h" 
#include "..\..\AeroDyn_Interface_Wrapper\eigen\Eigen\Dense"

using namespace Eigen;

void RenderBladeNodes(sf::RenderWindow& wnd, const std::vector<double>& bladeNodePositions, const Vector3d& hubPos, double pixelsPerMeter, int nNodes)
{

	sf::CircleShape shape(2.0);

	wnd.clear();

	for (int i = 0; i < nNodes; i++)
	{
		Vector3d nodePos(&bladeNodePositions[i * 3]); // set the node position from a pointer to the beginning of an array
		Vector3d relNodePos = nodePos - hubPos; // relative to the center position of the hub
		Vector3d scaledRelNodePos = relNodePos * pixelsPerMeter;
		Vector3d scaledHubPos = hubPos * pixelsPerMeter;
		Vector3d screenNodePos = scaledHubPos + scaledRelNodePos;

		// just take the y and z coordinates
		sf::Vector2f pos(screenNodePos.y(), screenNodePos.z());

		// use e^(-x) to map the x distance of the node from hub pos to a color value
		// between 0 - 255
		double deltax = nodePos.x() - hubPos.x();

		sf::Color clr;

		if (deltax > 0) {
			// make it blue
			unsigned int val = unsigned int(255.0 * (1.0 - exp(-deltax * 0.1)));

			clr = sf::Color(255 - val, 255 - val, 255);
		}
		else {
			// make it red
			unsigned int val = unsigned int(255.0 * (1.0 - exp(deltax * 0.1)));

			clr = sf::Color(255, 255 - val, 255 - val);
		}

		shape.setFillColor(clr);

		shape.setPosition(pos);
		wnd.draw(shape);
	}

	wnd.display();
}