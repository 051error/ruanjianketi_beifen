using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using WarehouseGridSimulation.Core.Geometry;

namespace WarehouseGridSimulation.Core.Map
{
    /// <summary>
    /// 地图加载与合法性校验器。
    /// </summary>
    public static class MapLoader
    {
        private static readonly JsonSerializerOptions JsonOptions = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true,
            WriteIndented = true
        };

        public static bool Validate(MapData data, out List<string> errors)
        {
            errors = new List<string>();
            if (data == null)
            {
                errors.Add("地图数据为空");
                return false;
            }

            if (data.Width <= 0 || data.Height <= 0)
            {
                errors.Add($"地图尺寸非法: Width={data.Width}, Height={data.Height}，必须均大于 0");
            }

            if (data.CellSizeMeters <= 0.05f)
            {
                errors.Add($"网格尺寸非法: CellSizeMeters={data.CellSizeMeters}，必须大于 0.05 米");
            }

            var obstacleSet = new HashSet<GridPoint>();
            if (data.Obstacles != null)
            {
                foreach (var obs in data.Obstacles)
                {
                    if (obs.X < 0 || obs.X >= data.Width || obs.Y < 0 || obs.Y >= data.Height)
                    {
                        errors.Add($"障碍物坐标越界: ({obs.X}, {obs.Y}) 超出 [0..{data.Width - 1}, 0..{data.Height - 1}]");
                    }
                    else
                    {
                        obstacleSet.Add(new GridPoint(obs.X, obs.Y));
                    }
                }
            }

            var stationIdSet = new HashSet<int>();
            if (data.Stations != null)
            {
                foreach (var st in data.Stations)
                {
                    if (!stationIdSet.Add(st.Id))
                    {
                        errors.Add($"存在重复的站点 ID: {st.Id}");
                    }

                    if (st.X < 0 || st.X >= data.Width || st.Y < 0 || st.Y >= data.Height)
                    {
                        errors.Add($"站点 #{st.Id} ({st.Name}) 坐标越界: ({st.X}, {st.Y})");
                    }
                    else if (obstacleSet.Contains(new GridPoint(st.X, st.Y)))
                    {
                        errors.Add($"站点 #{st.Id} ({st.Name}) 位于障碍物栅格: ({st.X}, {st.Y})");
                    }
                }
            }

            var robotIdSet = new HashSet<int>();
            var robotPosSet = new HashSet<GridPoint>();
            if (data.InitialRobots != null)
            {
                foreach (var r in data.InitialRobots)
                {
                    if (!robotIdSet.Add(r.Id))
                    {
                        errors.Add($"存在重复的机器人 ID: {r.Id}");
                    }

                    var p = new GridPoint(r.X, r.Y);
                    if (r.X < 0 || r.X >= data.Width || r.Y < 0 || r.Y >= data.Height)
                    {
                        errors.Add($"机器人 #{r.Id} 初始坐标越界: ({r.X}, {r.Y})");
                    }
                    else if (obstacleSet.Contains(p))
                    {
                        errors.Add($"机器人 #{r.Id} 初始位置位于障碍物中: ({r.X}, {r.Y})");
                    }
                    else if (!robotPosSet.Add(p))
                    {
                        errors.Add($"机器人 #{r.Id} 初始位置与另一机器人重叠: ({r.X}, {r.Y})");
                    }
                }
            }

            return errors.Count == 0;
        }

        public static (GridMap Map, MapData Data) LoadFromJson(string json)
        {
            if (string.IsNullOrWhiteSpace(json))
                throw new ArgumentException("JSON 内容为空", nameof(json));

            var data = JsonSerializer.Deserialize<MapData>(json, JsonOptions);
            if (!Validate(data, out var errors))
            {
                throw new InvalidOperationException($"地图数据校验失败:\n - " + string.Join("\n - ", errors));
            }

            var map = new GridMap(data.Width, data.Height, data.CellSizeMeters);
            if (data.Obstacles != null)
            {
                foreach (var obs in data.Obstacles)
                {
                    map.SetObstacle(new GridPoint(obs.X, obs.Y), true);
                }
            }

            if (data.Stations != null)
            {
                foreach (var st in data.Stations)
                {
                    var type = StationType.Waiting;
                    if (string.Equals(st.Type, "Pick", StringComparison.OrdinalIgnoreCase))
                        type = StationType.Pick;
                    else if (string.Equals(st.Type, "Drop", StringComparison.OrdinalIgnoreCase))
                        type = StationType.Drop;

                    map.AddStation(new Station(st.Id, st.Name, type, new GridPoint(st.X, st.Y)));
                }
            }

            return (map, data);
        }

        public static (GridMap Map, MapData Data) LoadFromFile(string filePath)
        {
            if (!File.Exists(filePath))
                throw new FileNotFoundException($"找不到地图配置文件: {filePath}");

            string json = File.ReadAllText(filePath);
            return LoadFromJson(json);
        }

        /// <summary>
        /// 创建默认的标准仓储仿真地图 (30x30 栅格，每个栅格对应真实物理尺寸 0.55 米，无障碍物清空地图，仅配置单个初始机器人)。
        /// </summary>
        public static (GridMap Map, MapData Data) CreateDefaultWarehouseMap()
        {
            // 创建默认的标准仓储仿真地图并配置上货、双排卸货与等待区站点。
            int width = 30;
            int height = 30;
            float cellSize = 0.55f;

            var data = new MapData
            {
                Name = "Warehouse30x30CleanMap",
                Width = width,
                Height = height,
                CellSizeMeters = cellSize,
                Obstacles = new List<PointDto>(), // 清空地图障碍物
                Stations = new List<StationDto>(),
                InitialRobots = new List<RobotInitialDto>()
            };

            // 1. 上货区配置：1x1 栅格，位于地图下方正中间 (X=15, Y=27)
            data.Stations.Add(new StationDto { Id = 1, Name = "上货区", Type = "Pick", X = 15, Y = 27 });

            // 2. 卸货区配置：两排 1x1 栅格，每排 3 个等距分布 (X=3, 15, 27)，中间间隔 3 行栅格 (第 1 排 Y=2，第 2 排 Y=6)
            data.Stations.Add(new StationDto { Id = 2, Name = "卸货区-1", Type = "Drop", X = 3, Y = 2 });
            data.Stations.Add(new StationDto { Id = 3, Name = "卸货区-2", Type = "Drop", X = 15, Y = 2 });
            data.Stations.Add(new StationDto { Id = 4, Name = "卸货区-3", Type = "Drop", X = 27, Y = 2 });
            data.Stations.Add(new StationDto { Id = 5, Name = "卸货区-4", Type = "Drop", X = 3, Y = 6 });
            data.Stations.Add(new StationDto { Id = 6, Name = "卸货区-5", Type = "Drop", X = 15, Y = 6 });
            data.Stations.Add(new StationDto { Id = 7, Name = "卸货区-6", Type = "Drop", X = 27, Y = 6 });

            // 3. 等待区配置：大小为 2 行 10 列 (共 20 格)，位于上货区左侧 (X=5..14, Y=26..27)，上货区位于等待区右侧第一列第二行
            int waitId = 10;
            for (int r = 1; r <= 2; r++)
            {
                int y = 25 + r; // 第 1 行 Y=26，第 2 行 Y=27
                for (int c = 1; c <= 10; c++)
                {
                    int x = 4 + c; // 第 1..10 列对应 X=5..14
                    data.Stations.Add(new StationDto
                    {
                        Id = ++waitId,
                        Name = $"等待位-{r}-{c}",
                        Type = "Waiting",
                        X = x,
                        Y = y
                    });
                }
            }

            // 单机初始机器人配置于上货区 (15, 27)，朝向正上方直面卸货区
            data.InitialRobots.Add(new RobotInitialDto { Id = 1, X = 15, Y = 27, Orientation = "Up" });

            string json = JsonSerializer.Serialize(data, JsonOptions);
            return LoadFromJson(json);
        }
    }
}
