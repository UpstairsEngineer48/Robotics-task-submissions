#!/usr/bin/env python3

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from task3.srv import PrimeFactors


class PrimeFactorClient(Node):

    def __init__(self):

        super().__init__('prime_factor_client')

        self.client = self.create_client(
            PrimeFactors,
            'prime_factors'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):

            self.get_logger().info(
                'Waiting for service...'
            )

    def send_request(self, number):

        req = PrimeFactors.Request()

        req.number = number

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        return future.result()


def main(args=None):

    rclpy.init(args=args)

    node = PrimeFactorClient()

    number = int(
        input("Enter a number: ")
    )

    response = node.send_request(number)

    print("Prime factors:",[1]+list(set((response.factors))))

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
