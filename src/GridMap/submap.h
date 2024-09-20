#ifndef SUBMAP_H
#define SUBMAP_H

#include "Eigen/Core"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gridmap.h"
#include "value_conversion_tables.h"
#include "probability_grid_range_data_inserter.h"

namespace AVP
{
namespace mapping
{
    
static const int max_semantic_data_in_submap = 20;

class Submap
{   
    public:
    Submap(const Eigen::Vector3d& local_pose, std::unique_ptr<GridMap> grid, 
            ValueConversionTables* conversion_tables);

    void InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                            ProbabilityGridRangeDataInserter* inserter);

    Eigen::Vector3d local_pose() const { return local_pose_; }
    int num_accumulated_semantic_data() const { return num_accumulated_semantic_data_; }
    void set_num_accumulated_semantic_data(const unsigned int num){num_accumulated_semantic_data_=num;}
    bool insertion_finished() const { return insertion_finished_; }
 
    void set_insertion_finished(bool insertion_finished) { insertion_finished_ = insertion_finished; }

    const GridMap* grid() const { return grid_.get(); }

    pcl::PointCloud<pcl::PointXYZ> data_;
    private:
    // MapLimits limits_;
    std::unique_ptr<GridMap> grid_;                     // 用于保存子图具体内容的对象
    const Eigen::Vector3d local_pose_;                  // submap的局部坐标系原点
    unsigned int num_accumulated_semantic_data_ = 0;    // 子图中插入的数据数量（帧数？）
    bool insertion_finished_ = false;                   // 标志子图是否已经构建完成，是否需要继续更新该子图
    ValueConversionTables* conversion_tables_;          // 转换表？？？
};


/**
 * 当前正在维护的submap
 */
class ActiveSubmaps
{
    public:

    ActiveSubmaps()
    {
        range_data_inserter.reset(new ProbabilityGridRangeDataInserter);
    }

    std::vector<std::shared_ptr<const Submap>> InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                                                                  const Eigen::Vector3d& pose_estimated);

    std::unique_ptr<GridMap> CreateGrid(const Eigen::Vector3d& origin);

    void AddSubmap(const Eigen::Vector3d& origin);

    std::vector<std::shared_ptr<const Submap>> submaps() const;

    private:
    std::vector<std::shared_ptr<Submap>> submaps_;
    ValueConversionTables conversion_tables_;
    // 插入器对象
    std::unique_ptr<ProbabilityGridRangeDataInserter> range_data_inserter;
};


} // namespace mapping
} // namespace AVP


#endif