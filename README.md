# stsmapgen_forSiv3D

The map generator was inspired by [yurkth/stsmapgen](https://github.com/yurkth/stsmapgen).
This is a port for use with [OpenSiv3D](https://github.com/Siv3D/OpenSiv3D).

# Demo

![demo](https://github.com/eightgamedev/stsmapgen_forSiv3D/assets/47023171/eea6b1f1-455b-49af-8e5b-f4e19010171f)

# Method

1. Set two points as start and end point.
2. Make a circle with two points as its diameters.
3. Set points within a circle by Poisson disk sampling.
4. Find the shortest path using Dijkstra's algorithm.
5. (permanently) Increase the cost of one edge in the path. 
6. Repeat steps No.4 to No.5.

# License

stsmapgen_forSiv3D is provided under the MIT License. Please see the LICENCE file for details.

# Library

stsmapgen_forSiv3D uses the following MIT licensed library.
- [OpenSiv3D](https://github.com/Siv3D/OpenSiv3D)

# Reference

- [stsmapgen](https://github.com/yurkth/stsmapgen)