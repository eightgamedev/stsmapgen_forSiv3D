# include <Siv3D.hpp> // Siv3D v0.6.13

// 辺の情報
struct Edge
{
	// 行先
	int to;

	// コスト
	int cost;
};

constexpr long long INF = (1LL << 60);

using Graph = Array<Array<Edge>>;

// { distance, from }
using Pair = std::pair<double, int>;

/// @brief ほどよい距離で重ならない点群を生成します。
/// @param circle 点群を生成する範囲
/// @param radius 点群の点の間の最小距離（目安）
/// @param clip true の場合、範囲外の点を切り取ります。
/// @return 生成された点群
Array<Vec2> GenerateRandomPointsInCircle(const Circle& circle, const Vec2 startPoint, const Vec2 endPoint, double radius)
{
	Array<Vec2> points;
	Rect rect = circle.boundingRect().asRect();
	PoissonDisk2D pd{ rect.size, radius};

	for (const auto& point : pd.getPoints())
	{
		const Vec2 pos = (point + rect.pos);

		if (not circle.contains(pos) || pos.distanceFrom(startPoint) < radius || pos.distanceFrom(endPoint) < radius)
		{
			continue;
		}

		points << pos;
	}

	// 始点と終点を追加
	points << startPoint << endPoint;

	return points;
}

/// @brief 点群からグラフを生成します。
/// @param triangles ドロネー三角形の配列
/// @param points 点群
/// @param subdiv 
/// @return Array<Array<Edge>>型のグラフ
Graph CreateGraphFromDelaunay(const Array<Triangle>& triangles, const Array<Vec2>& points, Subdivision2D& subdiv) {
	int n = points.size();
	Graph graph(n);

	for (const Triangle& triangle : triangles) {
		for (int i = 0; i < 3; ++i) {
			for (int j = i + 1; j < 3; ++j) {
				const auto nearestVertexIDsrc = subdiv.findNearest(triangle.asPolygon().vertices()[i]);
				const auto nearestVertexIDdest = subdiv.findNearest(triangle.asPolygon().vertices()[j]);
				const auto src = subdiv.getVertex(nearestVertexIDsrc.value());
				const auto dest = subdiv.getVertex(nearestVertexIDdest.value());
				Line line = { src, dest };
				int cost = line.length();
				int srcIndex = std::distance(points.begin(), std::find(points.begin(), points.end(), src));
				int destIndex = std::distance(points.begin(), std::find(points.begin(), points.end(), dest));

				if (cost > 0) {
					graph[srcIndex].push_back({ destIndex, cost });
					graph[destIndex].push_back({ srcIndex, cost });
				}
			}
		}
	}

	return graph;
}

/// @brief 経路上の辺のコストをランダムで1つ選び増加します。
/// @param graph グラフ
/// @param points 全点群 
/// @param path 既に計算済みの経路 
/// @param increaseAmount 増加するコスト量
void IncreaseEdgeCost(Graph& graph, const Array<Vec2>& points, const Array<int>& path, int increaseAmount) {

	// 経路上の辺からランダムに1つ選ぶ
	int randomIndex = Random<int>(0, path.size() - 2);
	int srcIndex = path[randomIndex];
	int destIndex = path[randomIndex + 1];

	// 辺のコストを増加
	for (auto& edge : graph[srcIndex]) {
		if (edge.to == destIndex) {
			edge.cost += increaseAmount;
		}
	}
}

/// @brief ダイクストラ法で最短経路を計算します。
/// @param graph グラフ
/// @param distances 頂点数と同じサイズ, 全要素 INF で初期化しておく
/// @param startIndex 開始地点のインデックス 
/// @param targetIndex 目的地点のインデックス
/// @param path 最短経路
void Dijkstra(const Graph& graph, Array<long long>distances, int startIndex, int targetIndex, Array<int>& path)
{
	// 直前の頂点を記録する配列
	Array<int> p(distances.size(), -1);

	// 「現時点での最短距離, 頂点」の順に取り出す priority_queue
	// デフォルトの priority_queue は降順に取り出すため std::greater を使う
	std::priority_queue<Pair, Array<Pair>, std::greater<Pair>> q;
	q.emplace((distances[startIndex] = 0), startIndex);

	while (!q.empty())
	{
		const long long distance = q.top().first;
		const int from = q.top().second;
		q.pop();

		// 最短距離でなければ処理しない
		if (distances[from] < distance)
		{
			continue;
		}

		// 現在の頂点からの各辺について
		for (const auto& edge : graph[from])
		{
			// to までの新しい距離
			const long long d = (distances[from] + edge.cost);

			// d が現在の記録より小さければ更新
			if (d < distances[edge.to])
			{
				// 直前の頂点を記録
				p[edge.to] = from;

				q.emplace((distances[edge.to] = d), edge.to);
			}
		}
	}

	// targetIndex に到達できれば最短経路を再構築する
	if (distances[targetIndex] != INF)
	{
		for (int i = targetIndex; i != -1; i = p[i])
		{
			path.push_back(i);
		}

		std::reverse(path.begin(), path.end());
	}
}

void addPath(Graph& graph, Array<long long>& distances, Array<Vec2> points, Array<Array<int>>& paths)
{
	Array<int> path;
	// ダイクストラ法で最短経路を計算
	Dijkstra(graph, distances, points.size() - 2, points.size() - 1, path);
	paths.push_back(path);

	// 経路上の辺のコストを増加
	IncreaseEdgeCost(graph, points, path, 10000);
}


void Main()
{
	const String applicationName = U"stsmapgen_forSiv3D";
	const String version = U"v1.0.0";
	Scene::SetBackground(Palette::White);
	Window::SetStyle(WindowStyle::Sizable);
	Window::SetTitle(U"{} {}"_fmt(applicationName, version));
	Scene::Resize(1920, 1009);
	Scene::SetResizeMode(ResizeMode::Keep);

	const Font font{ FontMethod::MSDF, 48, Typeface::Bold };
	const Font middleFont{ FontMethod::MSDF, 24, Typeface::Bold };

	// 始点と終点
	const Vec2 startPoint{ 1100, 850 };
	const Vec2 endPoint{ 1100, 150 };

	// 始点と終点を結ぶ線分を直径とする円
	const Vec2 areaCenter = (startPoint + endPoint) / 2;
	const double areaRadius = (startPoint - endPoint).length() / 2;
	const Circle areaCircle{ areaCenter, areaRadius };

	// 点群を生成する範囲
	// stretched(10) で余白を持たせないと、ドロネー三角形分割の際、範囲外参照になる
	const RectF areaRect = areaCircle.boundingRect().stretched(10);

	// 点群の点の間の最小距離
	constexpr double distance = 80;

	// 円の中に点群を生成
	Array<Vec2> points = GenerateRandomPointsInCircle(areaCircle, startPoint, endPoint, distance);

	// ドロネー三角形分割のインスタンス
	Subdivision2D subdiv{ areaRect, points };

	// ドロネー三角形分割の三角形リスト
	Array<Triangle> triangles;

	// ドロネー三角形分割の計算
	subdiv.calculateTriangles(triangles);

	// ドロネー三角形分割からグラフを生成
	Graph graph = CreateGraphFromDelaunay(triangles, points, subdiv);

	// ダイクストラ法に用いる変数
	Array<Array<int>> paths;
	Array<int> path;
	Array<long long> distances(points.size(), INF);

	// PerlinNoiseで古い紙質を表現
	PerlinNoise noise;
	size_t oct = 5;
	double persistence = 0.5;
	Rect mapArea = areaRect.asRect().stretched(100);
	Image mapImage{ mapArea.size, Palette::Burlywood };
	for (auto p : step(mapImage.size()))
	{
		// ノイズを生成
		double n = noise.normalizedOctave2D0_1((p / 128.0), oct, persistence);

		// ノイズの値に基づいて色を生成
		Color color = ColorF{ 0.64 + 0.36 * n, 0.48 + 0.52 * n, 0.27 + 0.73 * n }.toColor();

		mapImage[p] = color;
	}
	Texture mapTexture{ mapImage };

	Texture startTexture{ Emoji{ U"😀" } };
	Texture goalTexture{ Emoji{ U"😈" } };
	Texture skullTexture{ Emoji{ U"💀" } };
	Texture goldTexture{ Emoji{ U"💰" } };
	Texture questionTexture{ Emoji{ U"❓" } };
	Array<Texture> otherTextures{ skullTexture, goldTexture, questionTexture };
	double textureScale = 0.20;


	// GUI
	Vec2 checkBoxPos{ 50, 150 };
	bool drawCircle = true;
	bool drawPoints = true;
	bool drawTriangles = true;
	bool drawPaths = true;
	bool drawPathPoints = true;
	bool drawInGameStyle = false;
	while (System::Update())
	{
		if (drawInGameStyle)
		{
			mapTexture.drawAt(areaCenter);
			mapArea.drawFrame(10, Palette::Olive);
		}

		if (drawCircle)
		{
			areaCircle.drawFrame(2, Palette::Black);
		}

		if (drawTriangles)
		{
			for (const auto& triangle : triangles)
			{
				triangle.drawFrame(2, Palette::Black);
			}
		}

		if (drawPaths)
		{
			for (const auto& path : paths)
			{
				for (size_t i = 0; i < path.size() - 1; ++i)
				{
					Line{ points[path[i]], points[path[i + 1]] }.draw(2, Palette::Red);
				}
			}
		}

		if (drawPoints)
		{
			for (const auto& point : points)
			{
				Color color = (point == startPoint || point == endPoint) ? Palette::Red : Palette::Black;
				Circle{ point, 5 }.draw((point == startPoint || point == endPoint) ? Palette::Red : Palette::Black);
			}
		}

		if (drawPathPoints)
		{
			for (const auto& path : paths)
			{
				for (const auto& index : path)
				{
					Color color = (points[index] == startPoint || points[index] == endPoint) ? Palette::Red : Palette::Black;
					Circle{ points[index], 5 }.draw(color);
				}
			}
		}

		if (drawInGameStyle)
		{
			for (const auto& path : paths)
			{
				for (size_t i = 0; i < path.size() - 1; ++i)
				{
					Circle circleFrom{ points[path[i]], 10 };
					Circle circleTo{ points[path[i + 1]], 10 };
					Line lineOriginal = { circleFrom.center, circleTo.center };

					// lineOriginalと円周の交点を求める
					Optional<Array<Vec2>> pointFrom = circleFrom.intersectsAt(lineOriginal);
					Optional<Array<Vec2>> pointTo = circleTo.intersectsAt(lineOriginal);
					if (pointFrom && pointTo)
					{
						Line line{ pointFrom.value()[0], pointTo.value()[0] };
						line.drawArrow(2, { 10, 10 }, Palette::Olive);
					}
				}
			}
			for (const auto& path : paths)
			{
				for (const auto& index : path)
				{
					const auto& point = points[index];
					if (point == startPoint)
					{
						startTexture.scaled(textureScale).drawAt(point);
					}
					else if (point == endPoint)
					{
						goalTexture.scaled(textureScale).drawAt(point);
					}
					else
					{
						otherTextures[index % 3].scaled(textureScale).drawAt(point);
					}
				}
			}
		}

		if (SimpleGUI::Button(U"add Path", Vec2{ 50, 100 }, 100))
		{
			addPath(graph, distances, points, paths);
		}

		if (SimpleGUI::Button(U"reset Path", Vec2{ 170, 100 }, 100))
		{
			paths.clear();
			distances = Array<long long>(points.size(), INF);
			graph = CreateGraphFromDelaunay(triangles, points, subdiv);
		}

		SimpleGUI::CheckBox(drawCircle, U"draw Circle", checkBoxPos, 100);
		SimpleGUI::CheckBox(drawTriangles, U"draw Triangles", checkBoxPos.movedBy(0, 50), 100);
		SimpleGUI::CheckBox(drawPaths, U"draw Paths", checkBoxPos.movedBy(0, 100), 100);
		SimpleGUI::CheckBox(drawPoints, U"draw Points", checkBoxPos.movedBy(0, 150), 100);
		SimpleGUI::CheckBox(drawPathPoints, U"draw Path Points", checkBoxPos.movedBy(0, 200), 100);
		SimpleGUI::CheckBox(drawInGameStyle, U"draw in Game style", checkBoxPos.movedBy(0, 250), 100);

		font(applicationName).draw(50, 20, Palette::Black);
		middleFont(U"@eightgamedev").draw(mapArea.br().movedBy(-300, -80), Palette::Black);
		middleFont(U"inspired by @yurkth").draw(mapArea.br().movedBy(-300, -50), Palette::Black);
	}
}
