#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "fset"
#define AWS_CMD_USAGE "fset"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
