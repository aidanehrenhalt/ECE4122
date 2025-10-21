// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "controls.hpp"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}

/* Original
// Initial position : on +Z
glm::vec3 position = glm::vec3( 0, 0, 5 ); 
// Initial horizontal angle : toward -Z
float horizontalAngle = 3.14f;
// Initial vertical angle : none
float verticalAngle = 0.0f;
// Initial Field of View
float initialFoV = 45.0f;

float speed = 3.0f; // 3 units / second
// float mouseSpeed = 0.005f; // Original Code
float mouseSpeed = 0.00025f; // Adjusted mouse sensitivity
*/

// ACE
// Camera Rotational Movement
float radius = 10.0f;
float theta = 0.0f;
float phi = 3.14f / 4.0f;

// Camera Movement Speed
float radialSpeed = 5.0f;
float rotationSpeed = 2.0f;

void computeMatricesFromInputs(){

	// glfwGetTime is called only once, the first time this function is called
	static double lastTime = glfwGetTime();

	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);

	/* Original 
	// Get mouse position
	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);

	// Reset mouse position for next frame
	glfwSetCursorPos(window, 1024/2, 768/2);

	// Compute new orientation
	horizontalAngle += mouseSpeed * float(1024/2 - xpos );
	verticalAngle   += mouseSpeed * float( 768/2 - ypos );
	

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle), 
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)

	// Right vector
	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f/2.0f), 
		0,
		cos(horizontalAngle - 3.14f/2.0f)
	);
	
	// Up vector
	glm::vec3 up = glm::cross( right, direction );

	// Move forward
	if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
		position += direction * deltaTime * speed;
	}
	// Move backward
	if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
		position -= direction * deltaTime * speed;
	}
	// Strafe right
	if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
		position += right * deltaTime * speed;
	}
	// Strafe left
	if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
		position -= right * deltaTime * speed;
	}
	);*/

	// ACE
	// W Key - Move Closer (Decrease Radius)
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		radius -= radialSpeed * deltaTime;
		if (radius < 1.0f) radius = 1.0f; // Prevent getting too close
	}

	// S Key - Move Further (Increase Radius)
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		radius += radialSpeed * deltaTime;
	}

	// A Key - Rotate Left
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		theta += rotationSpeed * deltaTime;
	}

	// D Key - Rotate Right
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		theta -= rotationSpeed * deltaTime;
	}

	// Up Arrow Key - Rotate Up
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
		phi += rotationSpeed * deltaTime;
		if (phi > 3.14f - 0.1f) phi = 3.14f - 0.1f; // Prevent flipping
	}
	
	// Down Arrow Key - Rotate Down
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
		phi -= rotationSpeed * deltaTime;
		if (phi < 0.1f) phi = 0.1f; // Prevent flipping
	}

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 position = glm::vec3(
		radius * sin(phi) * cos(theta), // X
		radius * sin(phi) * sin(theta), // Y
		radius * cos(phi) // Z
	);
	
	// Look at Origin
	glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);
	
	// Up Vector (World Up)
	glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f);

	// Original - float FoV = initialFoV;// - 5 * glfwGetMouseWheel(); // Now GLFW 3 requires setting up a callback for this. It's a bit too complicated for this beginner's tutorial, so it's disabled instead.
	float FoV = 45.0f;

	// Projection matrix : 45� Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);

	/* // Camera matrix - Original
	ViewMatrix       = glm::lookAt(
								position,           // Camera is here
								position+direction, // and looks here : at the same position, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );
	*/

	// ACE
	ViewMatrix = glm::lookAt(
		position,
		target,
		up
	);

	// For the next frame, the "last time" will be "now"
	lastTime = currentTime;
}