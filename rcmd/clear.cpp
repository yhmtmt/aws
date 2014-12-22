#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "clear"
#define AWS_CMD_USAGE "clear"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
