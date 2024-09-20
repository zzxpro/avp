#include "submap.h"

namespace AVP
{
namespace mapping
{
    /**
     * 在构造Submap之前，需先提供一个GridMap的对象，在构造函数中通过std::move赋予成员变量grid_
     * @param local_pose        给出子图的位姿
     * @param grid              指示子图实际用于保存数据的对象
     * @param conversion_tables 0~32766预映射成浮点数的转换表
     */
    Submap::Submap(const Eigen::Vector3d& local_pose, std::unique_ptr<GridMap> grid, 
            ValueConversionTables* conversion_tables):local_pose_(local_pose),conversion_tables_(conversion_tables)
    {
        grid_ = std::move(grid);
    }

    /**
     * 将semantic_data数据插入到grid_对象中，inserter是一个辅助工具，具体负责插入数据的方式
     */
    void Submap::InsertSemanticData(const pcl::PointCloud<pcl::PointXYZ>& semantic_data,
                                    ProbabilityGridRangeDataInserter* inserter)
    {
        inserter->Insert(semantic_data, grid_.get());

        data_ += semantic_data;
        set_num_accumulated_semantic_data(num_accumulated_semantic_data()+1);
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertSemanticData(
        const pcl::PointCloud<pcl::PointXYZ>& semantic_data, const Eigen::Vector3d& pose_estimated)
    {
        // 在拥有两个子图后，第一个子图总比第二个子图多max_semantic_data_in_submap个scan
        // 1. 如果维护的子图为空 或 最新的子图中语义点数量超过最大值 则添加新的submap
        if (submaps_.empty() || submaps_.back()->num_accumulated_semantic_data() == max_semantic_data_in_submap)
        {
            // 新子图的初始位姿，为起始semantic_data的坐标，即num_accumulated_semantic_data帧，只取第一帧的初始位姿
            AddSubmap(pose_estimated);
        }
        // 2. 一次将Data插入进容器submaps_的子图中
        for(auto elem : submaps_)
        {
            elem->InsertSemanticData(semantic_data, range_data_inserter.get());
        }
        // 3. 容器submap_中最旧的submap中语义点数量过多，则该submap之后不再插入新的语义点
        if (submaps_.front()->num_accumulated_semantic_data() == 2*max_semantic_data_in_submap)
        {
            submaps_.front()->set_insertion_finished(true);
        }

        return submaps();
    }

    /**
     * @param origin        新建子图的原点坐标
     */
    void ActiveSubmaps::AddSubmap(const Eigen::Vector3d& origin)
    {
        // 1. 先检查容器submaps_中子图数量，如果不只有一个子图，删除最旧的子图
        if (submaps_.size()>=2)
        {
            submaps_.erase(submaps_.begin());
        }
        // 2. 构造一个新的submap,它将作为一个新子图用于插入数据，构建时调用CreateGrid函数为该对象提供一个保存栅格占用信息的存储结构
        submaps_.push_back(std::unique_ptr<Submap>(new Submap{origin, std::unique_ptr<GridMap>(CreateGrid(origin).release()),
         &conversion_tables_}));
    }

    std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
    {
        return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),submaps_.end());
    }

    /**
     * 用于为子图创建栅格信息存储结构
     */
    std::unique_ptr<GridMap> ActiveSubmaps::CreateGrid(const Eigen::Vector3d& origin)
    {
        // 在获取子图尺寸和分辨率信息之后，就构建一个 GridMap
        constexpr int kInitialSubmapSize = 100;
        float resolution = 0.05; // param: grid_options_2d.resolution
      
        MapLimits limit{resolution, 
                    origin.head<2>().cast<double>() + 0.5 * kInitialSubmapSize *resolution * Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)};

        return std::unique_ptr<GridMap>(
            new GridMap(limit, &conversion_tables_)
            );      
    }



} // namespace mapping
} // namespace AVP
