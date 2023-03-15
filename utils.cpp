#include <array>
#include <iterator>
#include <string>
#include <algorithm>
#include <sstream>
#include "utils.h"
#include "hand_models.h"
#include "settings.h"



template <class T, std::size_t N>
ostream& operator<<(ostream& o, const array<T, N>& arr)
{
	copy(arr.cbegin(), arr.cend(), ostream_iterator<T>(o, " "));
	return o;
}


// int main()
// {
//     Eigen::Quaterniond q = Eigen::Quaterniond(1.0,2.0,3.0,4.0);
// 	print_quat(q);
//     return 0;
// }