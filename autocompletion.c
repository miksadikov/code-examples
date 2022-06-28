#include "autocompletion.h"

#include <readline/history.h>
#include <readline/readline.h>

#define SIZE_MAX 128
#define MAX_CMDS_MUM 256
#define MAX_CMD_LEN 64

static char** character_name_completion(const char*, int, int);
static char* character_name_generator(const char*, int);
static char* escape(const char*);
static int quote_detector(char*, int);

char* character_names[MAX_CMDS_MUM];
char cmds_list[MAX_CMDS_MUM][MAX_CMD_LEN];

void autocompletion_init(void) {
  rl_attempted_completion_function = character_name_completion;
  rl_completer_quote_characters = "'\"";
  rl_completer_word_break_characters = " ";
  rl_char_is_quoted_p = &quote_detector;
}

static char** character_name_completion(const char* text, int start, int end) {
  rl_attempted_completion_over = 1;
  return rl_completion_matches(text, character_name_generator);
}

static char* character_name_generator(const char* text, int state) {
  static int list_index, len;
  char* name;

  if (!state) {
    list_index = 0;
    len = strlen(text);
  }

  while ((name = character_names[list_index++])) {
    if (rl_completion_quote_character) {
      name = strdup(name);
    } else {
      name = escape(name);
    }

    if (strncmp(name, text, len) == 0) {
      return name;
    } else {
      free(name);
    }
  }

  return NULL;
}

static char* escape(const char* original) {
  size_t original_len;
  int i, j;
  char *escaped, *resized_escaped;

  original_len = strlen(original);

  if (original_len > SIZE_MAX / 2) {
    errx(1, "string too long to escape");
  }

  if ((escaped = malloc(2 * original_len + 1)) == NULL) {
    err(1, NULL);
  }

  for (i = 0, j = 0; i < original_len; ++i, ++j) {
    if (original[i] == ' ') {
      escaped[j++] = '\\';
    }
    escaped[j] = original[i];
  }
  escaped[j] = '\0';

  if ((resized_escaped = realloc(escaped, j)) == NULL) {
    free(escaped);
    resized_escaped = NULL;
    err(1, NULL);
  }

  return resized_escaped;
}

static int quote_detector(char* line, int index) {
  return (index > 0 && line[index - 1] == '\\' &&
          !quote_detector(line, index - 1));
}

void remove_unused_symbol(char* str, char unused) {
  char *src, *dst;
  for (src = dst = str; *src != '\0'; src++) {
    *dst = *src;
    if (*dst != unused)
      dst++;
  }
  *dst = '\0';
}

void set_all_autocompletions(void) {
  DIR* d;
  FILE* fp;
  struct dirent* dir;
  char* cmdlst_file;
  int i, cmds_num = 0;
  char full_path[255];

  d = opendir("/usr/bin/");
  if (d) {
    while ((dir = readdir(d)) != NULL) {
      if (strstr(dir->d_name, ".cmdlst")) {
        full_path[0] = '\0';
        strcat(full_path, "/usr/bin/");
        cmdlst_file = dir->d_name;
        strcat(full_path, cmdlst_file);
        fp = fopen(full_path, "r");
        if (fp == NULL) {
          continue;
        }
        while (fgets(&cmds_list[cmds_num][0], MAX_CMD_LEN, fp)) {
          cmds_list[cmds_num][strlen(&cmds_list[cmds_num][0]) - 1] = '\0';
          character_names[cmds_num] = &cmds_list[cmds_num][0];
          cmds_num++;
          if (cmds_num > MAX_CMDS_MUM) {
            printf(
                "error: maximum number (%d) of commands for the autocompletion "
                "has been exceeded!\n",
                MAX_CMDS_MUM);
            break;
          }
        }

        fclose(fp);
      }
    }
    closedir(d);
    character_names[cmds_num] = NULL;
  }
}
