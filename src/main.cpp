
#include "SFML/System.hpp"
#include "SFML/Graphics.hpp"
#include <iostream>
#include <cstdlib>
#include "astar.h"

#define TILE_WIDTH 16
#define TILE_HEIGHT 16
#define WIDTH 80
#define HEIGHT 60


using namespace std;
using namespace astar;


void drawScreen(sf::RenderWindow& window, vector<sf::Sprite>& sprites, const vector<vector<int>>& map);
vector<sf::Sprite> loadImages(vector<sf::Texture>* textures);
void clearPathFromMap(vector<vector<int>>& map);


int main() {
	// SFML setup
	sf::RenderWindow window(sf::VideoMode(TILE_WIDTH * WIDTH, TILE_HEIGHT * HEIGHT), "A star Path Finder");
	vector<sf::Texture> textures;
	vector<sf::Sprite> sprites = loadImages(&textures);

	Astar pathFinder = Astar();
	pathFinder.setCosts(10, 14);
	Point2D start;
	Point2D target;

	vector<vector<int>> map;
	map.resize(HEIGHT);
	for (int ii = 0; ii < HEIGHT; ii++) {
		map.at(ii).resize(WIDTH, 0);
	}


	drawScreen(window, sprites, map);

	// main loop
	sf::Event event;
	while (window.isOpen()) {
		while (window.pollEvent(event)) {
			switch (event.type) {

			case sf::Event::Closed:
				window.close();
				break;

			case sf::Event::MouseButtonPressed:
				if (event.mouseButton.button == sf::Mouse::Button::Left) {
					// toggle wall at mouse location
					Point2D wall;
					wall.x = event.mouseButton.x / TILE_WIDTH;
					wall.y = event.mouseButton.y / TILE_HEIGHT;
					if (map.at(wall.y).at(wall.x) == 1) {
						map.at(wall.y).at(wall.x) = 0;
					}
					else {
						map.at(wall.y).at(wall.x) = 1;
					}
				}
				else if (event.mouseButton.button == sf::Mouse::Button::Right) {
					// set/move start point
					if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
						if (start.x != -1) {
							map.at(start.y).at(start.x) = 0;
						}
						start.x = event.mouseButton.x / TILE_WIDTH;
						start.y = event.mouseButton.y / TILE_HEIGHT;
						map.at(start.y).at(start.x) = 2;
					}
					// set/move target point
					else {
						if (target.x != -1) {
							map.at(target.y).at(target.x) = 0;
						}
						target.x = event.mouseButton.x / TILE_WIDTH;
						target.y = event.mouseButton.y / TILE_HEIGHT;
						map.at(target.y).at(target.x) = 3;
					}
				}
				drawScreen(window, sprites, map);
				break;

			case sf::Event::KeyPressed:
				if (event.key.code == sf::Keyboard::Return || event.key.code == sf::Keyboard::LControl) {
					clearPathFromMap(map);
					if (start.x != -1 && target.x != -1) {
						map.at(start.y).at(start.x) = 2;

						vector<vector<bool>> matrix;
						matrix.resize(HEIGHT);
						for (int ii = 0; ii < HEIGHT; ii++) {
							matrix.at(ii).resize(WIDTH, 0);
						}

						for (unsigned int yy = 0; yy < map.size(); yy++) {
							for (unsigned int xx = 0; xx < map.at(yy).size(); xx++) {
								if (map.at(yy).at(xx) == 1) {
									matrix.at(yy).at(xx) = true;
								}
								else {
									matrix.at(yy).at(xx) = false;
								}
							}
						}

						pathFinder.setMatrix(matrix);
						auto finalPath = pathFinder.findPath(start, target);
						if (finalPath != nullptr) {
							for each (auto p in *finalPath) {
								map.at(p.y).at(p.x) = 2;
							}
						}
						drawScreen(window, sprites, map);
					}
				}
				if (event.key.code == sf::Keyboard::R) {
					// randomize walls on map
					for (unsigned int yy = 0; yy < map.size(); yy++) {
						for (unsigned int xx = 0; xx < map.at(yy).size(); xx++) {
							if (rand() % 2 == 0) {
								map.at(yy).at(xx) = 1;
							}
							else {
								map.at(yy).at(xx) = 0;
							}
						}
					}
					drawScreen(window, sprites, map);
				}
				if (event.key.code == sf::Keyboard::C) {
					// clear screen
					for (unsigned int yy = 0; yy < map.size(); yy++) {
						for (unsigned int xx = 0; xx < map.at(yy).size(); xx++) {
							map.at(yy).at(xx) = 0;
						}
					}
					drawScreen(window, sprites, map);
				}
				break;
			}
		}
	}
	return 0;
}


void drawScreen(sf::RenderWindow& window, vector<sf::Sprite>& sprites, const vector<vector<int>>& map) {
	window.clear();
	int spriteVal;
	for (unsigned int yy = 0; yy < map.size(); yy++) {
		for (unsigned int xx = 0; xx < map.at(yy).size(); xx++) {
			spriteVal = map.at(yy).at(xx);
			sprites.at(spriteVal).setPosition((float)xx * TILE_WIDTH, (float)yy * TILE_HEIGHT);
			window.draw(sprites.at(spriteVal));
		}
	}
	window.display();
}


vector<sf::Sprite> loadImages(vector<sf::Texture>* textures) {
	sf::Texture temp;

	// empty space
	temp.loadFromFile("./res/empty.bmp");
	textures->push_back(sf::Texture(temp));
	// wall / blocked position
	temp.loadFromFile("./res/wall.bmp");
	textures->push_back(sf::Texture(temp));
	// start point
	temp.loadFromFile("./res/start.bmp");
	textures->push_back(sf::Texture(temp));
	// target point
	temp.loadFromFile("./res/target.bmp");
	textures->push_back(sf::Texture(temp));

	// create sprites
	vector<sf::Sprite> sprites;
	sprites.push_back(sf::Sprite(textures->at(0)));
	sprites.push_back(sf::Sprite(textures->at(1)));
	sprites.push_back(sf::Sprite(textures->at(2)));
	sprites.push_back(sf::Sprite(textures->at(3)));

	return sprites;
}

void clearPathFromMap(vector<vector<int>>& map) {
	for (unsigned int yy = 0; yy < map.size(); yy++) {
		for (unsigned int xx = 0; xx < map.at(yy).size(); xx++) {
			if (map.at(yy).at(xx) == 2) {
				map.at(yy).at(xx) = 0;
			}
		}
	}
}