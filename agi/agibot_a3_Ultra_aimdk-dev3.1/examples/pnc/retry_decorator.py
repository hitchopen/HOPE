#!/usr/bin/env python
# -*- coding: utf-8 -*-

import functools
import time
from typing import Callable, TypeVar

T = TypeVar("T")


def retry(max_attempts: int = 3, delay: float = 1.0):
    """
    Retry decorator for retrying function execution upon failure

    Args:
        max_attempts: Maximum number of retry attempts, defaults to 3
        delay: Delay between retries in seconds, defaults to 1.0

    Usage example:
        @retry(max_attempts=5, delay=2.0)
        def some_function():
            # Code that might fail
            pass
    """
    if max_attempts < 1:
        raise ValueError("max_attempts must be at least 1")
    if delay < 0:
        raise ValueError("delay must be non-negative")

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> T:
            attempts = 0
            last_exception = None

            while attempts < max_attempts:
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    attempts += 1
                    last_exception = e

                    if attempts < max_attempts:
                        print(
                            f"Function {func.__name__} failed, retrying attempt {attempts}, error: {str(e)}"
                        )
                        time.sleep(delay)
                    else:
                        print(
                            f"Function {func.__name__} reached max retry attempts {max_attempts}, giving up"
                        )

            if last_exception is not None:
                raise last_exception
            raise RuntimeError("Function failed but no exception was captured")

        return wrapper

    return decorator


# Usage example
if __name__ == "__main__":

    @retry(max_attempts=3, delay=0.5)
    def test_function():
        import random

        if random.random() < 0.7:  # 70% rate fail
            raise ConnectionError("Connection failed")
        return "Success"

    try:
        result = test_function()
        print(f"Execution result: {result}")
    except Exception as e:
        print(f"Final execution failed: {str(e)}")
