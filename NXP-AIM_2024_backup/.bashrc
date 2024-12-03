# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi


# Showing git branches in ubuntu terminal
force_color_prompt=yes
color_prompt=yes
parse_git_branch() {
 git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}
if [ "$color_prompt" = yes ]; then
 PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[01;31m\]$(parse_git_branch)\[\033[00m\]\$ '
else
 PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w$(parse_git_branch)\$ '
fi
unset color_prompt force_color_prompt


# custom aliases
alias s='source ~/.bashrc'
alias open='gedit ~/.bashrc'
alias show='cat ~/.bashrc'
# alias stest='source ~/workspaces/test_ws/install/setup.bash'
# alias smodels='export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/mohit/workspaces/test_ws/src/self_driving_car_pkg/models:/home/mohit/my-local-gzmodels'
# alias cb='cd ~/cognipilot/cranium && colcon build --packages-select b3rb_ros_line_follower && source ~/.bashrc'
# alias sil_launch='ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1'
# alias launch_track='ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1_sample'
# alias launch_foxglove='ros2 launch electrode electrode.launch.py sim:=True'
# alias runner_node='ros2 run b3rb_ros_line_follower runner'
# alias detect_node='ros2 run b3rb_ros_line_follower detect'
# alias vectors_node='ros2 run b3rb_ros_line_follower vectors'
# alias teleop_node='ros2 run b3rb_ros_line_follower teleop'
# alias test_node='ros2 run b3rb_ros_line_follower test'
# alias tides='ros2 run b3rb_ros_line_follower tides'
# alias nxp_launch='cd ~/cognipilot/cranium/AIM_2024_launcher && python3 launch_sim.py start'
alias burger='export TURTLEBOT3_MODEL=burger'
alias waffle_pi='export TURTLEBOT3_MODEL=waffle_pi'
# COGNIPILOT_SETUP
alias cball='colcon build'
alias cb='colcon build --packages-select'

source /opt/ros/humble/setup.bash
# if [ -f $HOME/cognipilot/ws/zephyr/scripts/west_commands/completion/west-completion.bash ]; then
#   source $HOME/cognipilot/ws/zephyr/scripts/west_commands/completion/west-completion.bash
# fi
# if [ -f $HOME/cognipilot/gazebo/install/setup.sh ]; then
#   source $HOME/cognipilot/gazebo/install/setup.sh
# fi
# if [ -f $HOME/cognipilot/cranium/install/setup.sh ]; then
#   source $HOME/cognipilot/cranium/install/setup.sh
# fi
# if [ -f $HOME/cognipilot/ws/cerebri/install/setup.sh ]; then
#   source $HOME/cognipilot/ws/cerebri/install/setup.sh
# fi
# if [ -f $HOME/cognipilot/electrode/install/setup.sh ]; then
#   source $HOME/cognipilot/electrode/install/setup.sh
# fi
# source /usr/share/colcon_cd/function/colcon_cd.sh
# source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# export ROS_DOMAIN_ID=7
# export CMAKE_EXPORT_COMPILE_COMMANDS=ON
# export CCACHE_TEMPDIR=/tmp/ccache
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export PYTHONWARNINGS=ignore:::setuptools.installer,ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install

#source /usr/share/gazebo/setup.sh
#export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/home/mohit/.gazebo/models

# for turtlebot3 BTP (after installing ros2 -- no personal use)
# alias teleop='ros2 run turtlebot3_teleop teleop_keyboard'
# alias tbot3_rviz='ros2 launch turtlebot3_bringup rviz2.launch.py'
# alias empty_world='ros2 launch turtlebot3_gazebo empty_world.launch.py'
# alias traj_node='ros2 run btp_pkg trajectory_node'

export ROS_DOMAIN_ID=13 #turtlebot3

#inter iit work
alias sitl='cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic'
alias dds-agent='cd ~/PX4-Autopilot/build/ && MicroXRCEAgent udp4 -p 8888'
alias gcs='cd ~ && ./QGroundControl.AppImage'
alias offboard='ros2 launch px4_offboard offboard_velocity_control.launch.py'
source ~/workspaces/inter-iit_ws/install/setup.bash
source ~/workspaces/PX4_ws/install/setup.bash
source ~/BTP_ws/install/setup.bash
export GPG_TTY=$(tty)    # github gpg key
