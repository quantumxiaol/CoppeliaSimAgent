"""Runtime configuration loaded from environment variables."""

from __future__ import annotations

from pydantic import Field, SecretStr
from pydantic_settings import BaseSettings, SettingsConfigDict


class AgentConfig(BaseSettings):
    """Environment-driven settings for LLM agent and MCP runtime."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    llm_model_name: str = Field(default="qwen-max-latest", alias="LLM_MODEL_NAME")
    llm_model_base_url: str = Field(
        default="https://dashscope.aliyuncs.com/compatible-mode/v1",
        alias="LLM_MODEL_BASE_URL",
    )
    llm_model_api_key: SecretStr = Field(alias="LLM_MODEL_API_KEY")

    mcp_server_host: str = Field(default="127.0.0.1", alias="MCP_SERVER_HOST")
    mcp_server_port: int = Field(default=7777, alias="MCP_SERVER_PORT")
    mcp_server_transport: str = Field(default="stdio", alias="MCP_SERVER_TRANSPORT")


def load_agent_config() -> AgentConfig:
    """Load config from `.env` and process environment."""
    return AgentConfig()
