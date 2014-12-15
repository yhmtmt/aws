#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "go"
#define AWS_CMD_USAGE "go [<start time>] [<end time>]"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
