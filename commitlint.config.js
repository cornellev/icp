module.exports = {
  extends: ['@commitlint/config-angular'],
  rules: {
    'type-enum': [
      2,
      'always',
      ['chore', 'ci', 'docs', 'feat', 'fix', 'refactor', 'revert', 'test']
    ],
  }
};
