#include <vector>
#include <algorithm>
#include <numeric>

// 中值滤波器类模板
class MedianFilter {
public:
    // 构造函数
    MedianFilter(){}

    // 应用滤波器
    float calculateMedian(std::vector<int>& window) {
        if(window.empty())return 0;
#ifdef CALI_MODE
        int sum = std::accumulate(window.begin(), window.end(), 0);
        return sum / window.size();
#else
        // 测量
            int windowSize = window.size();
            // 对邻域内样本进行排序
            std::sort(window.begin(), window.end());
            // 获取中间位置的值
            int middleIndex = windowSize / 2;
            float median;
            if (windowSize % 2 == 0) {
                median = (float)(window[middleIndex - 1] + window[middleIndex]) / 2;
            } else {
                median = (float)window[middleIndex];
            }
            return median;
#endif
    }

    float calculateAverage(std::vector<int>& window) {
        int windowSize = window.size();
        
        // 计算窗口中元素的总和
        int sum = 0;
        for (int i = 0; i < windowSize; i++) {
            sum += window[i];
        }
        
        // 计算平均值
        float average = static_cast<float>(sum) / windowSize;
        
        return average;
    }
private:
    int windowSize_;  // 邻域窗口大小
};