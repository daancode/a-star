## A* algorithm [![Build Status](https://travis-ci.org/daancode/a-star.svg?branch=master)](https://travis-ci.org/da-an/SHA-1)
A* search algorithm written in C++ programming language.
 - requires compiler support for C++11

#### Usage example
```cpp
#include <iostream>
#include "source/AStar.hpp"

int main()
{
    AStar::Generator generator;
    // Set 2d map size.
    generator.setWorldSize({25, 25});
    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Generate path ... \n";
    // This method returns vector of coordinates from target to source.
    auto path = generator.findPath({0, 0}, {20, 20});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
}
```
#### Preview
![](http://i.imgur.com/rqvrs6G.png)
![](http://i.imgur.com/7ZH2A0d.png)
