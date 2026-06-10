#!/usr/bin/env python3

import rclpy
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from task3.srv import PrimeFactors


class PrimeFactorServer(Node):

    def __init__(self):

        super().__init__('prime_factor_server')
        self.srv = self.create_service(
            PrimeFactors,
            'prime_factors',
            self.callback
        )

        self.get_logger().info(
            'Prime Factor Server Ready'
        )

    def callback(self, request, response):

        n = request.number
        factors = []
        while n % 2 == 0:
            factors.append(2)
            n //= 2
        d = 3
        while n > 1:
            while n % d == 0:
                factors.append(d)
                n //= d
            d += 2
        response.factors = factors
        return response


def main(args=None):

    rclpy.init(args=args)

    node = PrimeFactorServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
