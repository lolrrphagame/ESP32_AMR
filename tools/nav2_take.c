/* Humble's Python subscription discards the per-sample RMW writer GID.
 * Take the Twist and its metadata together using the installed Humble ABI. */
#include <string.h>
#include <geometry_msgs/msg/detail/twist__struct.h>
#include <rcl/subscription.h>

int amr_take_twist(const rcl_subscription_t *sub, double components[6],
                   int64_t stamps[2], uint8_t gid[24])
{
    _Static_assert(RMW_GID_STORAGE_SIZE == 24, "unexpected RMW GID size");
    geometry_msgs__msg__Twist twist = {0};
    rmw_message_info_t info = {0};
    rcl_ret_t rc = rcl_take(sub, &twist, &info, NULL);
    if (rc == RCL_RET_SUBSCRIPTION_TAKE_FAILED) return 0;
    if (rc != RCL_RET_OK) return -1;
    double values[] = {twist.linear.x, twist.linear.y, twist.linear.z,
                       twist.angular.x, twist.angular.y, twist.angular.z};
    memcpy(components, values, sizeof(values));
    stamps[0] = info.source_timestamp;
    stamps[1] = info.received_timestamp;
    memcpy(gid, info.publisher_gid.data, RMW_GID_STORAGE_SIZE);
    return 1;
}
