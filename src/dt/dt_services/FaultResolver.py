import sys
sys.path.append('..')

from config.grid_config import GRID_CONFIG

class FaultResolver:
    """Class to resolve faults in the system based on different mitigation strategies."""

    def shift_origins(self, task, current_block_no, shift_thresholds=True):
        """Shift the origins of the grid cells."""
        task_config = task
        current_block = current_block_no

        # 2) from the current block, change two first rows in its config to the next block
        for block_no in range(  # Iterate over blocks
            current_block + 1,
            task_config[GRID_CONFIG.NO_BLOCKS]
            - 1,  # ... from the next block to the last block
        ):

            task_config[block_no][GRID_CONFIG.ORIGIN][GRID_CONFIG.x] = (
                task_config[  # Change the x-coordinate to the next block's x-coordinate
                    block_no + 1
                ][
                    GRID_CONFIG.ORIGIN
                ][
                    GRID_CONFIG.x
                ]
            )
            task_config[block_no][GRID_CONFIG.ORIGIN][GRID_CONFIG.y] = (
                task_config[  # Change the y-coordinate to the next block's y-coordinate
                    block_no + 1
                ][
                    GRID_CONFIG.ORIGIN
                ][
                    GRID_CONFIG.y
                ]
            )

            if shift_thresholds:
                # Change the timing threshold to the next block's threshold
                task_config[block_no][GRID_CONFIG.TIMING_THRESHOLD] = (
                    task_config[block_no + 1][GRID_CONFIG.TIMING_THRESHOLD]
                )

        # remove the last block and decrement the number of blocks
        task_config.pop(task_config[GRID_CONFIG.NO_BLOCKS] - 1)
        task_config[GRID_CONFIG.NO_BLOCKS] -= 1

        # fault resolved
        return True, task_config

    def use_stock(self, task, current_block_no, stock_no):
        """Use the stock to resolve the fault."""
        task_config = task
        current_block = current_block_no
        pick_stock_tried = stock_no

        # For block[j] try PICK_STOCK[i++]
        if pick_stock_tried < len(GRID_CONFIG.PICK_STOCK_COORDINATES):
            task_config[current_block + 1][GRID_CONFIG.ORIGIN][
                GRID_CONFIG.x
            ] = GRID_CONFIG.PICK_STOCK_COORDINATES[pick_stock_tried][
                GRID_CONFIG.ORIGIN
            ][
                GRID_CONFIG.x
            ]
            task_config[current_block + 1][GRID_CONFIG.ORIGIN][
                GRID_CONFIG.y
            ] = GRID_CONFIG.PICK_STOCK_COORDINATES[pick_stock_tried][
                GRID_CONFIG.ORIGIN
            ][
                GRID_CONFIG.y
            ]
            pick_stock_tried += 1
            # fault resolved
            return True, task_config, pick_stock_tried
        else:
            # fault unresolved
            return False, task_config, pick_stock_tried