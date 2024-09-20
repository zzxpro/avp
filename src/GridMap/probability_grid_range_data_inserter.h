#ifndef PROBABILITY_GRID_RANGE_DATA_INSERTER_H
#define PROBABILITY_GRID_RANGE_DATA_INSERTER_H

#include "probability_values.h"
#include "gridmap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

namespace AVP
{
    
namespace mapping
{

/**
 * 概率栅格插入：维护两个数组：hit_table, miss_table；保存的是空闲概率映射后的索引值。
 * 如果发生了hit事件，就从hit表中查找更新后的数据，如果发生了miss事件则查找miss表
 * 
 */
class ProbabilityGridRangeDataInserter
{
    public:
    ProbabilityGridRangeDataInserter();

    void Insert(const pcl::PointCloud<pcl::PointXYZ>& range_data, GridMap* grid);


    private:
        // 用于更新栅格单元的占用概率的查找表
        const std::vector<uint16> hit_table_;
        const std::vector<uint16> miss_table_;

};

} // namespace mapping
} // namespace AVP


#endif