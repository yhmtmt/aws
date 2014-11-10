#include <iostream>
#include "aws_cmd.h"

#define AWS_CMD "online"
#define AWS_CMD_USAGE "online {yes|no}"

int main(int argc, char ** argv)
{
  return aws_cmd(argc, argv, AWS_CMD);
}
